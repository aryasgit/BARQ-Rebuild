# P6-02 — Environment & Reward Spec (simulator-agnostic, code-ready)

> Phase P6 · verified against repo @ 0e5ddaf

## Objective
One env spec that compiles to ANY backend (MuJoCo CPU, MJX, Isaac Lab): model import + parity
proof, observation/action contract, reward table, termination, curriculum, domain randomization.
Exit = gates G6.2 (parity) and G6.3 (reward sanity via classical-gait replay).

## Prerequisites
- P6-01 G6.1 passed (backend chosen, skeleton rollout runs).
- Read first: `barq_control/barq_control/leg_kinematics.py` docstring (the exact model),
  D-014/D-016/D-018/D-019, `robot_params.yaml`. The URDF is the single source of truth.

## Architecture rule (what makes three tracks possible)
Env code splits into a **pure core** (`barq_rl/core/`: obs layout, reward terms, termination,
curriculum, DR sampling — plain numpy-style functions over arrays) and a **backend adapter**
(~100 lines each: stepping, state extraction, vectorization). Only adapters mention a simulator.
`rl_obs.py` (P6-04 §4) is imported by the core, not reimplemented.

## 1. Model import: URDF → MJCF
1. Export plain URDF: `xacro barq.urdf.xacro mode:=mock > /tmp/barq_rl.urdf`, then strip the
   `<ros2_control>` block (and any leftover `<gazebo>` tags) — 5-line XML filter, keep it in
   `barq_rl/tools/strip_urdf.py`.
2. Convert: `python -c "import mujoco; m = mujoco.MjModel.from_xml_path('/tmp/barq_rl.urdf');
   import mujoco; mujoco.mj_saveLastXML('barq.xml', m)"` (MuJoCo's URDF import; verify exact API
   at execution time). Visual `.dae` meshes are NOT supported — drop visuals for training
   (collisions are primitives already: body box + 12 mm foot spheres). Optional `.dae`→`.obj`
   conversion only if you want pretty videos.
3. **Manual fixups (the contract — same list applies to Isaac/USD import):**
   | # | Fixup | Value / source |
   |---|---|---|
   | a | Free base | wrap the imported tree in a body with `<freejoint/>` (URDF import welds the root to world) |
   | b | Actuators (none come from URDF) | 12 `<position>` actuators, **kp from bench ID once P3 lands; until then kp = 37.45 N·m/rad, joint damping/kv = 0.624 N·m·s** (derivation §1.1) |
   | c | forcerange | **±2.94 N·m** all 12 |
   | d | ctrlrange | hips ±0.785, knees ±1.57, **ankles [−2.2, 0]** — NOTE: the URDF/ros2_control upper for ankles is 1.57, which contradicts D-012's design judgment [−2.2, 0] (robot_params servo block + ik_node clamp are right). Use **[−2.2, 0]**. Flag the URDF for fixing; do not import its ankle upper. |
   | e | Joint damping/friction | keep URDF `damping 0.05`; add small `armature` 0.001 (stability; TBD from bench ID) |
   | f | Foot contacts | sphere r = 0.012 at tibia tip (0,0,−0.1); `condim 3` to start (4/6 only if skating is observed); `friction = 0.9 0.005 0.0001`; μ is DR'd 0.3–1.2 (D-018 sweep heritage) |
   | g | Contact filtering | defaults (parent-child excluded); add explicit exclude body↔coxa if the importer didn't |
   | h | Integrator/timestep | `timestep 0.002` (500 Hz physics), implicitfast; policy decimation 10 → 50 Hz; low-level tick 100 Hz = every 5 physics steps (§4) |
   | i | Keyframe `stance` | qpos = base (0, 0, **0.142**, quat for 4.5° nose-down — capture the SETTLED quat per §2.5) + q_stance (§3 table) |
   | j | Lidar mass | keep the 47 g `laser` body at (−0.04, 0, 0.066) — the URDF models the future lidar ON PURPOSE ("train on the robot that will exist") |

### 1.1 Actuator model (parameterized — bench numbers MUST drop in)
The real device is a **position servo** with internal control ≈ velocity = k·error, velocity cap
4.71 rad/s, torque cap 2.94 N·m, k ≈ 60/s (D-018, sim-calibrated; **P3 bench ID supersedes**).
MuJoCo `<position>` is torque-based: τ = kp·(ctrl − q) − kv·q̇, clipped to forcerange. Mapping
that reproduces the same envelope (derive, then VERIFY by step test):
```
kp = τ_max · k / v_max = 2.94 × 60 / 4.71 = 37.45 N·m/rad      (saturation error 4.71/60 = 0.0785 rad)
kv = τ_max / v_max     = 2.94 / 4.71      = 0.624 N·m·s         (no-load slew settles at ≈ v_max)
```
All four numbers live in ONE file, `barq_rl/config/actuator.yaml` (`k_class, v_max, tau_max,
armature, latency_ms`), loaded by every backend adapter. When P3 produces bench values (rise
time → k; measured torque-speed points if the rig allows), update the yaml, not code.
**Validation (required, scripted: `tools/check_actuator.py`):** single leg fixed in air —
(1) small step 0.05 rad: 10–90 % rise time ≈ 2.2/k (±15 % vs the same metric from
`diagnostics/sim_actuation_probe.py` in Gazebo / `st3215_diag.py` on the bench);
(2) large step 1.0 rad: mid-travel slope = 4.71 rad/s ± 5 % (the rectangular envelope's vertical
edge). The rectangular torque-speed envelope is the CURRENT approximation; triangle derating is
an optional refinement once a measured curve exists (then implement as per-step torque clamp in
the adapter, not via kp/kv).

## 2. Parity validation script — `barq_rl/tools/check_parity.py` (gate G6.2)
Runs against the compiled model through the backend's API. Asserts (exact numbers, computed from
the repo's own `leg_kinematics.py` — re-derive if the URDF ever changes):
1. **Total mass = 2.4946 kg** ± 1e-3 (1.42 body + 0.047 lidar + 4 × 0.2569 leg). Note: the
   "robot ≈ 2.448 kg" figure excludes the modelled lidar; the URDF total INCLUDES it.
2. Per-link masses: base 1.42, laser 0.047, coxa 0.0733, femur 0.1536, tibia 0.030 (×4 each).
3. Joint ranges as fixup (d); ctrlrange ankle upper == 0.0 (catches the URDF 1.57 import).
4. forcerange ±2.94, velocity-related caps per actuator.yaml; foot sphere r = 0.012 present.
5. **FK at stance**: set qpos to q_stance, `mj_forward`, foot-sphere centers in BODY frame must
   equal (tol 1e-4 m; values verified against `fk_exact` to <1e-7):
   | foot | x | y | z |
   |---|---|---|---|
   | FL | +0.125924 | +0.0926605 | −0.1297799 |
   | FR | +0.125924 | −0.0902784 | −0.1297782 |
   | RL | −0.125811 | +0.0922597 | −0.1497782 |
   | RR | −0.125811 | −0.0906792 | −0.1497799 |
   (= hip_offsets + (kx, side·LAT, −depth); depth 0.13 front / 0.15 rear, D-016 trim.)
   Also assert `fk_exact(0, 1.047531, −1.928768, +0.01744, +1) ≈ (0.01744, 0.0754692, −0.13)`
   ± 1e-5 — proves the q_stance constants below match `leg_kinematics.py` on THIS machine.
6. **Settle test**: drop from keyframe, run 2 s: base height **0.142 ± 0.005 m** (Gazebo
   measured 0.1418), pitch **4.5° ± 1.5° nose-down**. SIGN CONVENTION TRAP: D-016 writes
   "+4.5° nose-down", the kickoff writes "−4.5°" (aero convention); the URDF lidar mount
   carries `rpy="0 -0.0785 0"` to COUNTER the body pitch — use that as the sign cross-check.
   Resolution: **record the settled base quaternion and the projected gravity g* — they become
   the canonical orientation target** (§5 reward, §6 termination). Never hand-type the sign.

## 3. Observation vector — 45 dims, float32 (the contract; `rl_obs.py` implements it)
Joint order everywhere = ros2_control order **FL,FR,RL,RR × hip,knee,ankle**. WARNING (Q-005):
the URDF tree declares legs FL,RL,FR,RR and `/joint_states` order is not guaranteed — every
consumer maps **by joint name**, never by index. q_stance (rad), from the URDF initial values:
```
FL/FR: hip 0.0, knee 1.047531, ankle −1.928768      RL/RR: hip 0.0, knee 0.911998, ankle −1.652637
```
| idx | quantity | source (deploy) | units | scale (obs = raw × scale) | train noise |
|---|---|---|---|---|---|
| 0:3 | gravity unit vector, base frame | /imu/data quat → R̂ᵀ·(0,0,−1) | – | 1.0 | rot by σ=0.02 rad (TBD vs bench BNO085) |
| 3:6 | base angular velocity | /imu/data gyro | rad/s | 0.25 | σ=0.002 + quantize 0.001 (STATE mrad/s) |
| 6:18 | q − q_stance | /joint_states | rad | 1.0 | σ=0.005 + quantize 0.001 (STATE mrad) |
| 18:30 | q̇ | /joint_states | rad/s | 0.05 | σ=0.1 + quantize 0.01 (STATE 10 mrad/s) |
| 30:42 | previous action (post-clip, pre-scale) | internal | – | 1.0 | none |
| 42:45 | command (vx, vy, wz) | /cmd_vel | m/s, rad/s | (2.0, 2.0, 0.25) | none |
**NO base linear velocity** — unobservable on hardware. Optional variant (+3 dims at 45:48,
estimator planar vel, scale 2.0): only with the estimator's noise modelled (4–5 % scale error,
sim-measured; hardware-degraded value TBD from P4) AND only if the blind policy plateaus. Flag
in the run card. Optional gait-clock extension (+2 dims, sin/cos of a 2 Hz phase): default
**OFF** — keep the policy blind and the deployment clock-free.

## 4. Action space
- 12 outputs `a ∈ [−1, 1]` (tanh/clipped), **offsets around q_stance** (front/rear DIFFER — the
  D-016 trimmed stance, §3): `q_target = q_stance + 0.25 · a`, then clamp to ctrlrange
  (ankles [−2.2, 0]).
- Rates: policy 50 Hz. Low-level: the 20 ms action interval is split into two 10 ms ticks with
  **linear interpolation from the previous target to the new one** (matches the 100 Hz CMD
  stream the hardware runs; the rl_policy_node does the SAME interpolation — P6-04 §5; train =
  deploy or you ship a hidden filter). Physics 500 Hz → 5 substeps per low-level tick.
- Action delay: a DR'd buffer (table §8) delays the target by 0–30 ms before it reaches the
  servo model — re-center on the P3-measured latency at P6-04.

## 5. Rewards (per policy step; total = Σ wᵢ·rᵢ · dt where dt = 0.02 s for the −costs marked /s)
Base linear velocity for the tracking terms comes from the SIMULATOR state (privileged —
rewards exist only in sim; the obs stays blind). Starting weights — expect to retune via the
failure-signature table (P6-03 §4):
| term | formula | weight (start) |
|---|---|---|
| lin-vel tracking | exp(−‖v_cmd,xy − v_xy‖² / 0.25) | **1.0** |
| ang-vel-z tracking | exp(−(wz_cmd − wz)² / 0.25) | **0.5** |
| alive | 1 | **0.25** |
| torque | −Σ τᵢ² | **2e-4** |
| action rate | −‖aₜ − aₜ₋₁‖² | **0.01** |
| joint accel | −‖(q̇ₜ − q̇ₜ₋₁)/dt‖² | **2.5e-7** |
| foot slip | −Σ_{feet in contact} ‖v_foot,xy‖² | **0.05** |
| feet air time | Σ_feet (t_air − 0.2) at touchdown, only while ‖cmd‖ > 0.05 | **1.0** |
| height | −(h − **0.142**)² | **30** |
| orientation | −‖g_b − g*‖²  (g* = projected gravity at the SETTLED stance, §2.6 — encodes the 4.5° nose-down design; do NOT use upright [0,0,−1]) | **5.0** |
| termination | −2 on fall | 1 |
Air-time target 0.2 s = the classical trot prior (period 0.5 × (1 − duty 0.6)) — gait-shaping
without a clock. Below the 0.05 cmd deadband: air-time term off, tracking targets zero velocity
(standing must be cheap, not rewarded for stepping).

## 6. Termination & episode
- Fall: |roll| > 0.8 rad; pitch beyond ±1.0 rad (band is generous because the stance is
  DESIGNED 4.5° nose-down — compute pitch relative to g*, not to upright, or the margin is
  asymmetric); base height < 0.06 m.
- Timeout 20 s (1000 steps) = **truncation, not failure**: bootstrap the value (time-limit
  handling on = `bootstrap_on_timeout`/`infinite_horizon` in your trainer — silent reward bias
  if you skip this).
- Reset: keyframe stance + initial jitter (§8), commands resampled per episode (and every 5–10 s
  within episodes once curriculum stage ≥ 1).

## 7. Curriculum
| stage | vx (m/s) | vy | wz (rad/s) | pushes | advance when |
|---|---|---|---|---|---|
| 0 | ±0.10 | ±0.05 | ±0.3 | none | tracking err < 25 % over 50 evals |
| 1 | ±0.15 | ±0.08 | ±0.5 | vel impulse 0.3 m/s every 8 s | err < 20 % |
| 2 | ±0.25 | ±0.10 | ±0.8 | 0.5 m/s every 6 s | err < 20 % → done |
Track C stops at stage 1+pushes (P6-01 §4). Realized-speed context: the classical gait does
~60 % of cmd at duty 0.6 (D-019) — the policy should beat that long before stage 2 ends (G6.5).

## 8. Domain randomization (per-episode unless noted)
| parameter | range (A/B) | C-narrow | how |
|---|---|---|---|
| foot friction μ | 0.3 – 1.2 | 0.5 – 1.0 | pair/geom friction |
| base mass | ±15 % (≈ ±0.21 kg on 1.42) | ±10 % | body mass scale |
| CoM shift (base) | ±2 cm x/y | ±1 cm | body ipos offset (Q-014 pending — re-center on measurement) |
| motor kp | ±20 % | ±10 % | actuator gain scale |
| torque cap | ±10 % | ±5 % | forcerange scale |
| latency | 0 – 30 ms | 0 – 20 ms | action delay buffer (re-center on P3 HIL measurement) |
| sensor noise | obs table §3 (URDF IMU: gyro σ 0.002 rad/s, accel σ 0.02 m/s²; + protocol quantization) | same | added in obs builder, train only |
| push (curriculum) | §7 | §7 | base velocity impulse |
| initial pose jitter | joints ±0.05 rad, base z ±0.01, yaw ±π, base vel ±0.1 m/s | same | reset |
Rule from the doomsday protocol: DR ranges get CENTERED ON MEASURED VALUES as P3/P4 produce them
(bench kp, latency, mass/CoM) — width is for residual uncertainty, not ignorance you could fix.

## Acceptance gates
- **G6.2 — parity.** `check_parity.py` all-green on the chosen backend (and re-run on the cloud
  instance before any paid run). Settle test §2.6 numbers recorded in the research log together
  with the captured g* / quaternion.
- **G6.3 — reward sanity by classical replay.** The exact cross-check that env frames match the
  robot: drive the ENV with the classical gait —
  `gait.foot_targets(t, 0.15, 0, 0, hip_offsets)` → `ik_exact` per leg (repo code, imported) →
  q_target stream at 50 Hz → env actions `a = (q_target − q_stance)/0.25` (inverse action map).
  PASS: robot walks **+X** (it walking −X = frame/sign bug, the exact class D-014 fixed), realized
  vx 0.06–0.10 m/s (Gazebo gives ~60 % of 0.15), no termination over 10 s × 5 seeds, mean
  lin-vel tracking term > 0.4, total reward positive. Record realized speed — it is the G6.5
  baseline number in THIS env.

## Fallback ladder
A: MJCF via auto-import + fixups → B: importer fights (mesh/inertia errors): hand-write the MJCF
from `robot_params.yaml` numbers (the model is 13 bodies and a table of origins — a day, and the
parity script judges it, not aesthetics) → C: backend swap (Isaac URDF→USD importer, same fixup
contract + parity numbers) → D: G6.3 fails on frames after 2 attempts: bisect with single-leg FK
prints env-side vs `fk_exact` (the parity table isolates which leg/axis lies). Switch rungs after
2 failed attempts or 1 session.

## Rollback
Env-repo only; nothing deployed. Revert to the last commit where G6.2 passed (parity script in CI
keeps that floor).

## Artifacts → docs/05_RESEARCH_LOG.md
Parity output (mass/FK/settle numbers + captured g*), actuator step-test plots vs the Gazebo/bench
metric, G6.3 replay realized-speed (the in-env classical baseline), every fixup that differed from
the table above (those diffs are exactly what the next person needs).

## Escape hatch
If this spec's obs/action shape proves wrong (e.g., the policy needs the gait clock, or
estimator velocity), CHANGE THE SPEC FIRST (this file + `rl_obs.py` together, one commit),
retrain, re-export — never patch the deployed obs builder alone. The spec is the interface;
divergence between this table and `rl_obs.py` is the one bug this phase cannot survive.
