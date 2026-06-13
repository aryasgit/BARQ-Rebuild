# P6-01 — RL Strategy & the Three Compute Tracks

> Phase P6 · verified against repo @ 4ea53a0

## Objective
Decide HOW the locomotion policy gets trained (which compute track), lock what the policy is and
is not, and stand up the environment repo skeleton on the chosen backend. Exit = gate G6.1.

## What the policy IS (scope lock — read before arguing with the reward spec)
- A **blind velocity-tracking policy**: proprioception only (gravity vector from the IMU quat,
  gyro, joint pos/vel, previous action) + the `(vx, vy, wz)` command. Trot-like by prior, but the
  gait is learned, not scripted.
- A **drop-in replacement for gait_planner + ik_node**. Correction to the kickoff/overview graph
  ("publishes /joint_targets"): there is **no `/joint_targets` topic in the code**. The real chain
  is `gait_planner → /foot_targets → ik_node → /joint_group_position_controller/commands`
  (Float64MultiArray, 12 floats, ros2_control order **FL,FR,RL,RR × hip,knee,ankle** — see
  `barq_bringup/config/ros2_controllers.yaml`). The policy bypasses BOTH gait and IK and publishes
  joint positions **directly to `/joint_group_position_controller/commands`**. Everything below
  that topic (controller_manager 100 Hz → barq_hw CMD 100 Hz → Teensy 500 Hz superloop, deadman
  200 ms) is untouched.
- Deployed as **ONNX on the Jetson Orin Nano**, onnxruntime CPU (45-dim obs @ 50 Hz is trivial;
  TensorRT only if measured inference > 2 ms — P6-04).
- The classical stack **remains installed and selectable at a one-command switch** (P6-04 §6).

## What the policy is NOT (v1)
No vision/lidar in the obs. No stairs. No gallop/bound. No base-linear-velocity obs (unobservable
on hardware without leaning on the estimator's 4–5 % drift — optional variant only, P6-02 §3).
No torque control — the ST3215 is a position servo; actions are position targets, full stop.

## Prerequisites
- Repo at/after `4ea53a0`; read P6-02 before writing env code (this file only picks the track).
- `docs/02_DECISIONS.md` D-014/D-016/D-018/D-019 read. `robot_params.yaml` is geometry truth.
- Money/hardware question answered honestly: cloud budget? x86+RTX physically present? Otherwise
  the Mac mini (Apple Silicon) + Jetson are what exist.

## 1. Track selection flowchart
```
Is an x86+NVIDIA-RTX machine physically available to you for >= 1 week?
 ├─ YES → TRACK B (local Isaac Lab). Cheapest per-iteration learning loop.
 └─ NO → Is there budget (team cap, suggested ₹25,000 phase total) for rented GPU time?
     ├─ YES → TRACK C FIRST for 2–3 days as pipeline rehearsal (free, catches env bugs),
     │         then TRACK A (cloud GPU) for the real runs. Never debug rewards at ₹/hr.
     └─ NO  → TRACK C alone. Honest expectations in §4. Re-evaluate budget if C produces a
               policy that stands+steps in sim but transfers poorly (that's the signal one
               cloud run would likely close the gap) — see switch criteria §6.
```
The three tracks share EVERYTHING except the vectorization backend: the env spec (P6-02) is
simulator-agnostic, `rl_obs.py` (P6-04 §4) is shared verbatim, gates/eval/run-cards identical.
Switching tracks mid-phase loses backend glue code only.

## 2. Track A — rented cloud GPU (RunPod / Lambda / Vast.ai class)
**Provider class comparison criteria** (fill the table at execution time; do NOT trust remembered
prices — they move monthly):

| Criterion | Why it matters | How to check |
|---|---|---|
| ₹/GPU-hr, on-demand vs interruptible | dominates cost | provider pricing page, convert USD→₹ same day |
| Billing granularity (per-minute?) | short debug sessions | pricing page |
| Mid-GPU availability (see sizing) | queue time kills weekends | live console |
| Persistent volume ₹/GB-month | keep env+venv between sessions | pricing page |
| Egress fees | checkpoints/TB logs come back | pricing page |
| SSH + Docker support | our workflow is rsync+ssh | docs |
| Payment that works from India | hard blocker | try ₹500 top-up first |

**Instance sizing:** one **mid RTX-class GPU** (12–24 GB VRAM: 3090/4070Ti/4090/A10/L4 class) is
enough — 12 DoF, 45-dim obs, MLP ≤ [512,256,128], 4096 envs PPO fits in a few GB. ≥8 vCPU,
≥32 GB RAM, ≥60 GB disk. Multi-GPU buys nothing here. Interruptible/spot is fine ONLY with the
checkpoint discipline of P6-03 §5 (resume tested before the first long run).

**Backend on Track A:** Isaac Lab (PhysX, rsl_rl reference configs) is the default. If Track C
rehearsal already produced a working MJCF env, **MuJoCo/MJX on the rented GPU is the lower-risk
path** (same asset, same env core, only the vectorizer changes). Pick ONE; do not maintain both.

**Workflow (repo subset to ship):**
1. Ship: `barq_description/urdf/ + config/` (xacro→URDF export per P6-02 §1), the env repo
   (`barq_rl/`: env core, backend adapter, vendored `rl_obs.py`, parity script, train configs,
   requirements lock). Tarball or public git clone. **No SSH keys / secrets on the instance** —
   pull artifacts FROM the Mac side (`rsync instance:…/runs/ ./`), never push from the instance.
2. First 10 minutes on the instance: run the parity script (G6.2 must pass THERE — driver/version
   skew is real), then a 5-min throughput probe.
3. Data back after every session: latest + best checkpoints, tensorboard event files, the run
   card (§5). The instance is disposable at all times.

**Cost estimation method (never invent prices):**
```
hours      = N_steps_planned / (S_measured × 3600)        # S = env-steps/s from the 10-min probe
cost_run ₹ = rate_₹/hr × hours × 1.25                      # 1.25 = setup/eval/restart overhead
```
Planning numbers for N_steps (verify against your own curves after run 1): first-walk ≈ 100–200 M
env-steps; robust+DR ≈ 0.5–1.5 B. S on a mid RTX for this size is typically 10⁴–10⁵ env-steps/s —
**MEASURE, then update this file's TBD table.** After run 1, replace both numbers with measured.

**Budget guard (team policy caps — adjust knowingly, but write the new number down first):**
pilot run ≤ ₹1,500 · full run ≤ ₹6,000 · phase total ≤ ₹25,000 before a mandatory team review.
**Kill criteria** (stop the instance, no sunk-cost negotiation): projected cost > cap at the
10-min probe; OR no improvement in tracking-reward over the last 25 % of the planned steps; OR
the failure-signature table (P6-03 §4) says the fix is a config change — fix offline, relaunch.

| TBD (Track A) | How produced |
|---|---|
| rate_₹/hr (chosen provider+GPU) | provider page on launch day, recorded in run card |
| S_measured env-steps/s | 10-min probe, run card 001 |
| N_steps to first-walk / robust (ours) | tensorboard of runs 001/002 |

## 3. Track B — local x86 + RTX (if it materializes)
1. Install: Ubuntu 22.04+, NVIDIA driver + CUDA per Isaac Lab's support matrix **at execution
   time** (pin whatever you install in `barq_rl/PINNED.md`: driver, CUDA, isaac-sim, isaaclab,
   rsl_rl, torch versions — the approach is pinned by this doc, the numbers by that file).
2. Verify install with the stock Isaac Lab quadruped example BEFORE importing BARQ — separates
   "install broken" from "our env broken".
3. Asset path: URDF→USD via Isaac Lab's URDF importer; run the SAME parity assertions (P6-02 §2)
   through the Isaac API. Fix importer fixups (position-drive gains, ankle ctrl range) exactly as
   the MJCF fixup list — the list is the contract, the file format is incidental.
4. Same env core, same configs, same run cards as Track A. Electricity is the only marginal cost;
   the budget guard becomes a wall-clock guard (same kill criteria minus ₹).

## 4. Track C — Mac mini (Apple Silicon) + Jetson only
Honesty first: **this track trains smaller and slower and the result will be less robust.** It is
STILL worth executing — it rehearses 100 % of the pipeline (env, parity, reward, export, deploy,
gates) so that any later GPU access is spent on GPU-shaped problems only, and it can produce a
conservative low-speed policy that may pass the hardware gates.
- **Backend: MuJoCo CPU, native (`pip install mujoco`)** on the Mac. MJX exists, but JAX-on-Metal
  is experimental (verify at execution time; expect missing ops) — **treat CPU MuJoCo as the
  plan, MJX-on-Metal as a probe you time-box to 2 hours.**
- Scale: **64–256 envs** (python `multiprocessing` / `mujoco.rollout` threaded), policy net
  **[128,128]**, PPO via sb3 or cleanrl (P6-03 §3). Expect 10–50 M steps per run; with the ≥3k
  env-steps/s gate (P6-03) that is hours-to-a-day per run, and **days-to-weeks of iterations**
  to a usable conservative policy. The Jetson is for deployment, not training (6×A78AE + shared
  8 GB; don't split learner/actor across machines — complexity buys ~nothing at this size).
- PyTorch MPS for a [128,128] MLP is often SLOWER than CPU (dispatch overhead) — benchmark once,
  default to CPU, move on.
- Reduced curriculum: cap commands at vx ±0.15, wz ±0.5; narrower DR (P6-02 table, "C-narrow"
  column). A policy that walks at 0.10–0.15 m/s blind with modest pushes is the realistic target.

## 5. Reproducibility discipline (all tracks, non-negotiable)
Every run gets a **run card** committed to the repo at `training_runs/<id>.yaml` before the run
is declared done (checkpoints/TB files themselves stay OUT of git — rsync to the Mac, back up):
```yaml
id: 2026-06-20_A_003            # date_track_serial
commit: <repo sha>              # repo state the env/config came from
config_sha256: <hash>           # sha256 of the RESOLVED merged config dump (env+train yaml)
seed: 42
track: A | B | C
machine: runpod-4090 | mac-m2 | ...
backend: isaaclab-X.Y | mujoco-X.Y (+ python, torch versions)
env_count: 4096
steps_total: 5.2e8
wall_clock_h: 7.4
cost_inr: 1840                  # 0 for B/C
result: "walks 0.21 m/s, fails wz>0.6; tracking_err 14%"   # one honest line
artifacts: { ckpt: <path/best.pt>, tb: <path>, obs_stats: <path/obs_stats.npz> }
gates: { G6.4: fail, ... }
```
Seed + config snapshot per run; any conclusion quoted in docs needs the 3-seed rule (P6-03 §5).

## 6. Switch criteria between tracks
- C → A: C's policy stands+steps in sim and passes G6.3 reward sanity, but G6.4 needs more
  envs/DR than CPU affords (curves still improving at step budget), or it transfers poorly at
  P6-04 while the obs-gap report is clean → **budget ONE cloud run** at the pilot cap.
- A → C: payment/availability blocked for > 1 week → continue iterating on C; A resumes from C's
  configs unchanged.
- B → A: local GPU dies/leaves → ship the same env repo to a rented instance (workflow §2).
- Any → bypass: P6-03 escape hatch fires (no G6.4 within 3 runs at cap or 3 weekends) →
  `05_NO_RL_BYPASS.md` becomes the phase's main line; RL parks, seam stays documented.

## 7. Timeline expectations (calibrate after the first run, don't promise)
| Track | Setup | First walking policy in sim | G6.4-candidate |
|---|---|---|---|
| A | 1 evening | same day (GPU-hours: ~1–3) | 1–2 weekends |
| B | 0.5–1 day install | same day | 1–2 weeks of evenings |
| C | 1 day | 1–2 days | 1–3 weeks, may plateau short of full envelope |

## Acceptance gate
- **G6.1 — track chosen + skeleton rollout.** A run card `…_000.yaml` exists naming the chosen
  track and backend with the flowchart reasoning; the env repo skeleton executes a **1000-step
  random-policy rollout** on the chosen backend: obs batch shape `(N, 45)`, all values finite,
  robot visibly does *something* in the viewer/render, no NaN/crash; committed.

## Fallback ladder
A: chosen track per flowchart → B: if its backend install fights you > 2 sessions, swap backend
within the track (Isaac Lab ↔ MuJoCo) — the env spec doesn't care → C: drop one track tier
(B→A→C) → D: no track executable (no money, no Mac) → execute `05_NO_RL_BYPASS.md` only; record
the decision in the research log. Switch after 2 failed attempts or 2 sessions, whichever first.

## Rollback
Nothing touches the robot in P6-01/02/03. Rollback = delete the env repo branch; the deployed
stack is untouched until P6-04, where rollback is the one-command mode switch.

## Artifacts → docs/05_RESEARCH_LOG.md
Log: chosen track + why (flowchart leaf), backend versions pinned, throughput probe number,
run card 000/001 ids, any price/availability facts gathered (dated — they rot fast).

## Escape hatch
If this whole strategy is wrong for the situation you're in (e.g., a lab GPU cluster appears, or
compute prices collapsed): keep §5's run-card discipline and P6-02's env spec — they are
compute-agnostic — and rewrite only this file's tracks. The spec outlives the hardware market.
