# P6-04 — Sim2Real Transfer & Deployment

> Phase P6 · verified against repo @ 0e5ddaf

## Objective
Carry a G6.4/G6.5 policy from the env to the robot: re-center the env on measured hardware
values, export to ONNX with a mandatory parity proof, stand up the ONE new ROS node
(`rl_policy_node`) with its safety wrapper and the gait↔policy switch, climb the hardware ladder
(cradle → floor+spotter → free), and close the bag-driven iteration loop. Exit gates
**G6.7–G6.11**.

## Prerequisites
- **G6.4 + G6.5 passed** (envelope stated in the run card if reduced — the hardware gates bind
  at that envelope).
- **P3 complete**: bench actuator ID (G3.7) and the HIL latency numbers exist — or their absence
  is explicitly recorded and the DR ranges stay WIDE (§1 rule).
- **P4 complete through the drills**: G4.6 (stop ladder), G4.7 (fall detect), G4.8 (battery
  floor) — the same drills govern policy sessions; P4-CODE-1 telemetry live (`/imu/data` +
  `/barq/diag` publish on hardware).
- Read `phase-4-standing-walking/03_ESTIMATOR_AND_SAFETY.md` §5b — the stop ladder is the
  safety spine of this file.

## 1. Pre-transfer checklist (re-center the env on measured reality, THEN retrain/re-eval)
Doomsday rule (02 §8): DR width is for residual uncertainty, not for ignorance you could fix.

| env parameter | training value today | replace with | source / procedure |
|---|---|---|---|
| actuator k → kp/kv (`actuator.yaml`) | k = 60/s derived (02 §1.1) | bench-ID k (rise time → k = 2.2/t_rise) | P3 G3.7 step captures; `check_actuator.py` re-run vs bench overlay |
| torque-speed envelope | rectangular 2.94 / 4.71 | measured points (optional refinement) | P3 bench sweep if the rig allows (02 §1.1) |
| action-delay buffer | DR 0–30 ms | measured cmd→STATE round trip, DR = measured ± 10 ms | P3 HIL latency probe (03_HIL_VALIDATION TBD: full-chain added latency) |
| base mass / CoM | geometric-center inertial | measured CoM xyz | Q-014 measurement; edit URDF inertial origin, **re-run `check_parity.py`** |
| IMU noise σ (obs table) | URDF values (02 §8) | bench BNO085 σ + wander | P4-03 §2 wander test + a 60 s static bag |
| estimator drift (only if est-vel obs variant) | 4–5 % sim | hardware % | P4-03 §3 L-course (< 10 % bar) |
| foot friction μ center | 0.9, DR 0.3–1.2 | **no bench rig — keep wide** | honesty row: do not narrow what you didn't measure |
| ankle ctrlrange | [−2.2, 0] env-side fixup | confirm URDF flag resolved or fixup retained | 02 §1 fixup (d) |

After any re-center: re-run the G6.4/G6.5 **eval** (cheap). Retrain only if eval breaks; that
retrain is a normal P6-03 run (cards, caps, seeds).

## 2. Export: torch → ONNX + the mandatory parity test (gate G6.7)
1. `barq_rl/tools/export_policy.py` (~30 lines): load checkpoint, strip to the actor MLP, eval
   mode, `torch.onnx.export(actor, dummy(1,45), opset_version=17)` — **pin the opset in
   `PINNED.md`; verify your onnxruntime supports it at execution time**. Static batch 1, input
   `obs`, output `action`.
2. If running obs normalization was ON for this run (P6-03 §1.3): **fold mean/var into the
   graph** (prepend the affine, or bake into the first Linear). The parity test is what proves
   you did it right; `obs_stats` alone on disk is a deployment bug waiting.
3. Export also writes `model_meta.yaml` next to the .onnx: `{action_scale: 0.25, obs_dim: 45,
   rl_obs_sha256: <hash of rl_obs.py>, train_commit, run_id, opset}`. The node refuses to start
   on any mismatch (§5) — this is the normalization-skew kill switch.
4. **Parity script — `diagnostics/check_onnx_parity.py` (~20 lines, MANDATORY):**
   ```
   seed 0; obs = 100 vectors: 90 × Uniform[−3,3]^45 + 10 real rows from an eval rollout
   a_t = torch_actor(obs)  (fp32, CPU);  a_o = onnxruntime InferenceSession(...).run(obs)
   assert max|a_t − a_o| < 1e-5; print max delta + per-call latency p50/p99
   ```
   Run it in THREE places: training machine, the Mac, **the Jetson** (aarch64 onnxruntime CPU
   wheel — verify availability/version at execution time).

**G6.7 — export parity.** max |Δaction| < 1e-5 on training machine AND Jetson; Jetson latency
recorded (§3); `model_meta.yaml` hash matches the deployed `rl_obs.py`.

## 3. Jetson runtime
`pip install onnxruntime` (CPU EP). Benchmark: 1000 inferences, report p50/p99 — a ≤190k-param
MLP at 45 dims @ 50 Hz is trivial on the Orin Nano's A78AE cores; expect well under 1 ms.
**TensorRT only if measured p99 > 2 ms** (01's rule) — do not buy that complexity on a guess.
Pin `onnxruntime` version in `barq_rl/PINNED.md`. TBD row: the measured p50/p99.

## 4. The shared obs builder — `barq_control/barq_control/rl_obs.py`
ONE file used by training eval AND deployment. Divergence between this file and the 02 §3 table
is the bug this phase cannot survive (02's escape hatch): spec + file change together, one commit.
- Pure numpy, no ROS imports (importable inside `barq_rl`). Module constants: `OBS_DIM = 45`,
  `JOINT_ORDER` (FL,FR,RL,RR × hip,knee,ankle), `Q_STANCE` (the 02 §3 constants), `SCALES`.
- Signature:
  ```python
  def build_obs(joint_pos: dict, joint_vel: dict,      # name-keyed (Q-005: never by index)
                quat_xyzw, gyro_rad_s,                 # /imu/data fields
                prev_action: np.ndarray,               # (12,) post-clip pre-scale
                cmd: tuple) -> np.ndarray:             # (vx, vy, wz) -> (45,) float32
  ```
- Gravity: `g_b = −(third row of R(quat))` = −(2(xz−wy), 2(yz+wx), 1−2(x²+y²)). Do NOT trust the
  algebra as typed here — it is pinned by the unit tests below.
- Vendor sync: `barq_rl` ships a byte-identical copy for cloud instances;
  `barq_rl/tools/check_vendor_sync.py` asserts sha256 equality, and G6.7 re-checks the hash via
  `model_meta.yaml`.
- Unit tests (`barq_control/test/test_rl_obs.py`): (1) golden test — fixed inputs → a pinned
  45-vector literal committed with the file; (2) identity quat → g = (0,0,−1); (3) **g\*
  regression**: the settled quaternion captured at G6.2 (research log) → must reproduce the
  recorded g\* (kills the 02 §2.6 pitch-sign trap forever); (4) dict insertion-order scramble →
  identical output; (5) scale spot-checks per the 02 §3 table.

## 5. `rl_policy_node` — the one new ROS node (package `barq_control`, ~150 lines)
| item | spec |
|---|---|
| subscribes | `/joint_states` (100 Hz), `/imu/data` (100 Hz, P4-CODE-1), `/cmd_vel`, `/barq/diag` (1 Hz, fault byte [15]), `/fall_detected` (+ `/halt_gait` if the watchdog node runs), `/barq/control_mode` (§6) |
| publishes | `/joint_group_position_controller/commands` (Float64MultiArray, 12, ros2_control order) **@ 100 Hz**; optional `/rl/obs` + `/rl/action` when `debug_pub:=true` (for §8 bags) |
| params | `model_path`, `policy_rate 50.0`, `cmd_timeout 1.0`, `slew_per_tick 0.0471` (rad / 10 ms = 4.71 rad/s), `stale_hold_ms 40`, `stale_sit_ms 200`, `debug_pub false` |
| startup | validate `model_meta.yaml` (action_scale 0.25, obs_dim 45, rl_obs sha) — refuse on mismatch; wait for first `/joint_states` + `/imu/data`; **start from the MEASURED pose** (D-020's anti-lurch): blend measured → policy output over 1.0 s |
| loop | 100 Hz timer; every 2nd tick: `build_obs` → onnxruntime → post-process → new target; intermediate tick publishes the linear-interp midpoint — the SAME two-tick interpolation the env trained (02 §4); train = deploy, no hidden filter |
| post-process | clip a to [−1,1] → `q_target = Q_STANCE + 0.25·a` → safety wrapper below |

**Safety wrapper (ordered, all in-node, counters logged at 1 Hz):**
1. NaN/inf in action → hold last targets, error log, counter.
2. Hard joint-limit clamp to ctrlrange: hips ±0.785, knees ±1.57, **ankles [−2.2, 0]**.
3. Slew limit: ≤ 0.0471 rad per 10 ms tick per joint (= 4.71 rad/s, the servo's own cap —
   ≤ 0.0942 rad per 20 ms policy step).
4. Stale-obs watchdog: newest(`/joint_states`, `/imu/data`) age > **40 ms** → hold pose (freeze
   targets, torque on); > **200 ms** → **controlled sit**: interpolate 1.5 s to `q_sit`
   (computed at init via `ik_exact` at stance_height 0.10 — the G4.8 sit pose; imported, never
   hand-typed) and hold.
5. `/barq/diag` fault byte bit0/1/2 (servo-bus / IMU / power) → controlled sit + error. (1 Hz
   topic ⇒ up to 1 s latency — acceptable: the firmware acts on its own faults faster; bit3
   deadman means the firmware already cut torque.)
6. `/fall_detected` or `/halt_gait` True → HOLD (stop publishing; controller holds last
   position), same semantics as the gait stack (P4-03 §5c/§5d); operator runs the ladder.
7. `/cmd_vel` silent > `cmd_timeout` → cmd = (0,0,0) (the policy stands; zero-cmd behavior is
   trained, 02 §5) — mirrors `gait_planner`'s deadman.

The counters are gate currency: G6.8's "no limit slams" is read off them, not off vibes.

## 6. The gait↔policy switch (one command, deadman chain intact)
- Mode = latched topic `/barq/control_mode` (`std_msgs/String`, `"gait"` | `"policy"`,
  transient_local). `rl_policy_node` publishes only in `policy`; `gait_planner` gets a ~6-line
  mirror gate (same pattern as `honor_fall`) so it goes silent in `policy`. ONE publisher to the
  commands topic at any time, by construction.
- Switch procedure (both directions): zero `/cmd_vel`, robot at STAND →
  `ros2 topic pub --once /barq/control_mode std_msgs/String "{data: policy}"`. During the
  handover gap the controller holds last positions (HOLD semantics — safe). This is 01's
  promised one-command switch; classical remains installed as the A/B fallback.
- **Stop ladder under policy — P4's finding applies verbatim**: killing `rl_policy_node` does
  NOT trip the firmware deadman — ros2_control keeps streaming the last positions (exactly as
  killing gait+ik doesn't, P4-03 §5b). The ladder is identical with rung 2 reworded:

| rung | action | result |
|---|---|---|
| 1 | zero `/cmd_vel` | policy stands (trained zero-cmd) |
| 2 | kill `rl_policy_node` (or mode→`gait` with gait stack down) | **HOLD** — torque ON, not a stop |
| 3 | Ctrl-C the `real.launch.py` terminal | firmware deadman → **LIMP** (~200–300 ms) |
| 4 | master switch | power gone |

  Re-drill rung 3 ONCE with the policy stack live before any floor work (folded into G6.8);
  the G4.6 discipline (hand near the terminal whenever the robot is on the floor) carries over
  unchanged.

## 7. Hardware deployment ladder (same discipline as P4: cradle → floor+spotter → free)
**Step 0 — emulator HIL (zero robot):** `real.launch.py` against `teensy_emulator` (D-020 PTY) +
`rl_policy_node`. Verify: 100 Hz commands flow, mode switch both ways, and the watchdog paths —
pause/kill the emulator feed to fire the 40 ms hold and 200 ms sit branches. Hardware never
debugs what the PTY can.

- **G6.8 — cradle air-policy, 60 s.** Robot suspended, feet free. cmd 0 (30 s) then vx 0.05
  (30 s). PASS: ctrlrange-clamp hits = 0; slew-clamp hits < 1 % of ticks; zero fault bits;
  temp_max < 60 °C (P4-03 thresholds card); motion looks trot-like, no thrash. Note: unloaded
  legs are off-distribution (no contact) — pawing/marching is EXPECTED; this gate is sanity,
  not tracking. Plus: rung-3 re-drill (§6) passes < 300 ms.
- **G6.9 — floor, stand + low-cmd.** Pad, spotter, P4 floor rules. Stand 60 s, then vx ≤ 0.05
  wander: **2 min total, no fall — ×3 consecutive sessions-runs.**
- **G6.10 — the P4 regression table, re-run under policy.** Same protocol as P4-02 (tape
  course, 3 runs/row, fresh starts), same rows (surface × speed). PASS row-for-row vs the
  classical entries: realized % ≥ classical's; zero falls/assists/faults; |lat| and |heading|
  ≤ classical's measured; temp_max ≤ classical + 5 °C. This is the README's phase exit bar
  ("policy ≥ classical gait on the regression table").
- **G6.11 — push-recovery demo.** Floor + spotter: gentle hip-height hand pushes (same operator
  throughout, on video), at stand ×5 and at vx 0.10 ×5: **≥ 8/10 recover unassisted** (no fall,
  no hand save). Sim said ≥ 80 % (G6.4c) — this is its hardware echo, deliberately informal.

## 8. Iteration loop: bag → obs-gap → widen DR → retrain
1. Bag every hardware session: `ros2 bag record /joint_states /imu/data /cmd_vel /barq/diag
   /rl/obs /rl/action` (`debug_pub:=true`).
2. `diagnostics/obs_gap_report.py` (~60 lines): rebuild the obs stream from the bag **through
   `rl_obs.build_obs` itself** (same module — another skew killer), compare per-dim p1/p5/p50/
   p95/p99 against the training `obs_stats.npz` (P6-03 §5). Flag any dim where hardware p5/p95
   falls outside training p1/p99. Output: a markdown table per bag.
3. Widen DR only on the flagged dim-groups (ONE group per retrain — Protocol §2):

| flagged dims | DR knob to widen |
|---|---|
| 0:3 gravity | init orientation jitter, push magnitude |
| 3:6 gyro | sensor noise σ, pushes |
| 6:18 q | init joint jitter, motor-kp range |
| 18:30 q̇ | latency range, kp range, friction |
| 30:42 prev action | not a DR problem — the policy itself differs; investigate before retraining |
| 42:45 cmd | not DR — drive richer commands in the next session |

4. Retrain (a normal P6-03 run, caps apply) → re-export → G6.7 → ladder re-entry at G6.8.

## Acceptance gates
**G6.7** export parity · **G6.8** cradle 60 s · **G6.9** floor 2 min ×3 · **G6.10** regression
table policy ≥ classical · **G6.11** push demo ≥ 8/10. No skipping rungs, no exceptions for
"it worked in sim" (Protocol §3).

## Fallback ladder
A: fails at G6.8/G6.9 with clean counters → §8 loop (obs gap → one DR group → retrain) →
B: counters dirty (clamp/slew slams) → the policy is fighting the actuator model: re-verify §1
rows, re-run `check_actuator.py` vs bench, retrain with corrected `actuator.yaml` →
C: G6.10 close but not row-for-row → run BOTH modes mission-style for a session and judge: if
policy wins on falls/pushes but loses 5 % realized speed, take it to the team as a documented
trade — the gate stays unticked until a row-for-row pass or a recorded team override →
D: two §8 loops without progress → escape hatch. Switch after 2 failed attempts or 2 sessions.

## Rollback
One command: `/barq/control_mode → "gait"` (relaunch gait+ik if stopped). The classical stack is
never uninstalled; the policy node is additive and silent when not in `policy` mode. Reverting
the deployment = stop launching `rl_policy_node`. Keep all .onnx + meta + bags — they are the
iteration loop's memory.

## TBD table
| # | Unknown | Procedure |
|---|---|---|
| 1 | bench k → kp/kv | P3 G3.7 → `actuator.yaml` |
| 2 | cmd→STATE full-chain latency | P3 HIL probe → delay-buffer center |
| 3 | base CoM xyz | Q-014 → URDF inertial → parity re-run |
| 4 | BNO085 σ / wander | P4-03 §2 + 60 s static bag |
| 5 | Jetson ORT p50/p99 | §3 benchmark, 1000 runs |
| 6 | hardware obs-gap dims | first §8 report |

## Artifacts → docs/05_RESEARCH_LOG.md
G6.7 parity deltas + Jetson latency; the §1 checklist with measured values and what was
re-centered; counter summaries per cradle/floor run; the filled G6.10 table beside P4's classical
rows (that comparison is publication material); §8 gap reports + which DR group was widened and
why; run cards for every retrain. Commit + push per session.

## Escape hatch
If transfer fundamentally fails — two §8 loops, clean obs-gap reports, G6.8/G6.9 still failing —
the seam is almost certainly the actuator model, not the policy: go back to the bench, re-ID with
richer probes (loaded steps, chirps), rebuild `actuator.yaml`, and compare `check_actuator.py`
overlays against bench captures. If a third loop still fails: park RL with full artifacts (cards,
bags, gap reports — the publication record of an honest negative), adopt `05_NO_RL_BYPASS.md`,
and leave `rl_policy_node` + the mode switch in the tree: the next attempt deploys into the same
seam with zero new plumbing.
