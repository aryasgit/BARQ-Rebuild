# P6-05 — The No-RL Bypass: a classical ladder to a satisfying robot

> Phase P6 · verified against repo @ 4ea53a0

## Objective
Six independently shippable feedback/reflex rungs on the EXISTING gait stack that reach a
deployable, mission-capable robot even if RL never lands. Each rung: spec (file, ~lines,
params), effort, expected gain, acceptance gate. Exit = as many rungs as the mission needs;
the Protocol §6 finish line (c) is satisfied by **rung 2**.

## Relationship to P6 proper (read even if RL is going well)
- **Rungs 1–2 are worth doing EVEN IF RL succeeds**: they raise the classical baseline (making
  G6.10's row-for-row comparison honest and harder), and the classical stack is the permanent
  A/B fallback mode (P6-04 §6) — a better fallback is a better robot.
- This ladder is not a consolation prize. It is the control-theoretic answer to the SAME two
  measured problems RL targets: Q-013 (~60 % realized speed — swing-drag at the 20 mm front
  clearance ceiling) and Q-016 (duty-dependent yaw veer). RL attacks them by learning around
  the workspace ceiling; the ladder attacks them by closing loops the stack already has sensors
  for.
- Activation: by choice any time after P4, or automatically when the P6-03/P6-04 escape hatches
  fire (01 §6 "Any → bypass").

## Prerequisites
- P4 exited (G4.3+), stop-ladder drills standing (G4.6/G4.7 discipline at every floor session).
- Estimator qualified on hardware: the P4-03 §3 drift number is on record (< 10 % bar). Rungs
  1–2 EAT estimator output — if P4 parked the estimator out of spec, fix that first (P4-03
  fallbacks), or the loops below chase noise.
- P4-CODE-1 telemetry live (`/imu/data`, `/barq/diag`); R5 additionally needs the §5 load topic.

## 0. Ladder rules
1. Every rung is a ROS param, **default OFF** — the open-loop gait of P4 is always recoverable
   by relaunch. One rung enabled per measured run while qualifying (Protocol §2).
2. Sim-first where sim can host it: R1/R2/R4 are verified in Gazebo with the existing
   `diagnostics/sim_walk_metric.py` before metal; R3 needs a one-evening 5° ramp world (~20-line
   SDF); R5 is hardware-only (sim publishes no servo load) — cradle false-trigger check first.
3. All rungs live in `gait_planner_node.py` / `gait.py` (plus one read-only topic in `barq_hw`
   for R5). No firmware changes anywhere on this ladder.
4. Workspace truth that rules everything here: in-plane reach at the tibia stop (q3 = −2.2) is
   **0.1079 m** (D-019) — front apex depth at stand 0.13 / step 0.02 is 0.110 m, i.e. the front
   clearance ceiling is ~2 mm of margin. Any rung that touches step_height / stand_height /
   rear_raise must re-check `stand − step ≥ 0.1079 + margin` per leg pair (one-off check via
   `ik_exact` at the stroke extremes; ValueError/clamp = out of envelope).

## The ladder at a glance
| rung | what | file, ~lines | effort | expected gain | gate |
|---|---|---|---|---|---|
| R1 | body-velocity feedback | gait_planner_node, ~30 | 1–2 sessions | realized 60 % → ~85–90 % | G6.12 |
| R2 | yaw-rate feedback | gait_planner_node, ~20 | 1 session | kills Q-016 veer at duty 0.6 | G6.13 |
| R3 | pitch reflex (rear_raise trim) | gait_planner_node, ~25 | 1–2 sessions | ±5° slope tolerance | G6.14 |
| R4 | command shaping (accel ramps) | gait_planner_node, ~15 | 0.5 session | no lurch on cmd steps | G6.15 |
| R5 | stumble reflex (swing-load spike) | barq_hw +12, gait_planner ~30 | 2–3 sessions | 8–10 mm debris tolerance | G6.16 |
| R6 | convex-MPC-lite (literature, stretch) | none — a decision | 0.5 session reading | honesty: likely none needed | G6.17 |

## 1. R1 — body-velocity feedback (the Q-013 closer)
**Idea**: the estimator already measures realized vx/vy (`/odom_est` twist, 50 Hz, low-passed);
the gait commands ~60 % of what's asked (D-019). Close the loop with a slow stance-sweep scale
multiplier — the gait's stroke `sx = vx·period·duty` simply gets the corrected vx.
**Spec** (`gait_planner_node.py`, ~30 lines): subscribe `/odom_est`; params `speed_fb` (bool,
default False), `speed_fb_ki` (0.5 /s), `speed_fb_max` (1.8). Integrator on the speed ratio:
`m += ki·(|v_cmd| − |v_est|)/max(|v_cmd|, 0.05) · dt`, clamp `m ∈ [1.0, speed_fb_max]`, freeze
`m` when `|v_cmd| < 0.03` or within 2 s of walk start (estimator needs stance cycling); apply
`vx_eff = m·vx, vy_eff = m·vy` into `foot_targets`. Slow on purpose: the estimator is the
sensor (4–5 % sim / ≤10 % hw drift) — time constant ≥ 2 s, never a per-cycle correction.
**Honesty cap**: a longer stroke does not need more swing clearance, but it does raise swing
travel speed and drag exposure; expect saturation before m = 1.8 — the saturation point (reach
or torque, watch `/barq/diag` loads as m climbs) is TBD-1 and is itself a result.
**Gate G6.12** — realized speed **≥ 85 %** of cmd at vx 0.10 (tape-measured over the P4 5 m
course, median of 3 runs), no falls, |lat| within the P4 L2 band.

## 2. R2 — yaw-rate feedback (the Q-016 killer)
**Idea**: Q-016's veer flips sign with duty (0.55 straight / 0.6 fast+veer ≈ +0.32 rad/10 s
left). The IMU yaw rate is already in `/odom_est` (twist.angular.z, gyro passthrough). A P-term
on wz error feeds the EXISTING `wz` path in `foot_targets`, which already realizes differential
stance sweep L/R (`sx = (vx − wz·ny)·…`) — the mechanism is free, only the loop is new.
**Spec** (~20 lines, shares the R1 `/odom_est` subscription): params `yaw_fb` (default False),
`yaw_fb_kp` (0.5), `yaw_fb_max` (0.3 rad/s). `wz_eff = wz_cmd + kp·(wz_cmd − wz_imu)`, clamped
to ±`yaw_fb_max` of correction; small deadband (0.02 rad/s) against gyro noise.
**Gate G6.13** — duty **0.6** (the fast setting), vx 0.10, 5 m straight: **|heading drift|
< 0.05 rad** at the end, ×3 consecutive, no falls. Record before/after at duty 0.55 and 0.6
both — the Q-016 entry gets closed with this table.

## 3. R3 — pitch reflex (live rear_raise trim)
**Idea**: D-016's nose-down stance is open-loop; on a slope the body attitude drifts from
design. Trim `rear_raise` live from IMU pitch error vs the DESIGN attitude (the settled-g\*
convention — 02 §2.6's sign trap applies; never hand-type the sign, regression-test against the
captured quat per P6-04 §4 test 3).
**Spec** (~25 lines): subscribe `/imu/data` (pitch = `asin(clamp(2(wy−zx)))`, the P4-CODE-5
formula); params `pitch_reflex` (default False), `pitch_kp` (0.05 m/rad as integrator gain),
trim slew ≤ 5 mm/s, clamp `rear_raise ∈ [0.0, 0.035]` (rear apex at 0.035: depth 0.165, apex
0.145 — fold-safe; verify the stance extreme via `ik_exact` once, TBD-3).
**Authority honesty** (from D-016's measured 4.5° per 0.02 m): the clamp gives roughly
**−4.5°…+3.4°** of trim about design attitude. Downhill (needs nose-up trim) is fully covered
at 5°; uphill is authority-limited near +5° — expect partial compensation, and do NOT chase it
by contracting the front legs (front apex would violate the 0.1079 m fold floor at step 0.02,
rule §0.4).
**Gate G6.14** — plywood ramp at **±5°**: walks up and down at vx 0.05, ×3 each direction, no
falls/tip; measured |pitch error vs design| reduced **≥ 50 %** vs reflex-off baseline (bag both).

## 4. R4 — command shaping (accel ramps on /cmd_vel)
**Idea**: nav2 and teleop send step commands; the open-loop gait obeys instantly and lurches.
Slew-limit the command inside the gait node.
**Spec** (~15 lines in `_tick`): params `cmd_shaping` (default False), `max_accel` (0.2 m/s²),
`max_yaw_accel` (1.0 rad/s²) — defaults to verify in sim first. Note: **environmental** slowdown
is already handled — nav2's costmap inflation cost-scaling slows the commanded path near
obstacles (P5 config); R4 is about the ROBOT's dynamic limits, do not duplicate planner logic
here. Interaction: ramps add command latency — confirm nav2's controller tolerates it (TBD-5)
before declaring mission-ready.
**Gate G6.15** — cmd step 0 → 0.10: ×5 no falls, and max |pitch transient| in the 1 s after the
step (from `/imu/data` bags) reduced vs ramps-off; one P5-style nav2 mission completes with
ramps on.

## 5. R5 — stumble reflex (swing-leg load spike → clearance boost)
**Idea**: a swing leg should be unloaded; a mid-swing load spike on its servos = the foot hit
something. STATE frames already carry per-servo load at 100 Hz — it just stops at `/barq/diag`'s
1 Hz today.
**Spec**:
- `barq_hw` (+~12 lines, P4-CODE-1 pattern): publish `/barq/servo_load` (Float64MultiArray, 12,
  %, **50 Hz** decimated from STATE).
- `gait_planner_node.py` (~30 lines): params `stumble_reflex` (default False),
  `stumble_load_pct` (TBD-4), `stumble_boost` (1.5×, 2 cycles). The planner knows each leg's
  phase; during a leg's swing window, knee-servo load > threshold → next cycle: step_height ×
  boost **for that diagonal pair** + temporary `stand_height` +0.01.
- **Why stand_height must come along** (rule §0.4): at stand 0.13 the FRONT step_height ceiling
  is ~0.022 (0.13 − 0.1079) — there is no headroom to boost. stand 0.14 raises the front
  ceiling to ~0.032; rear has room regardless (rear depth 0.15+). The boost without the
  stand raise is a no-op exactly where stumbles matter most — this coupling IS the rung.
- Threshold procedure (TBD-4): bag 60 s cradle air-walk + 60 s floor walk; threshold = floor
  swing-window load p99 × 1.3. Cradle first: zero false triggers in 60 s air-walk before floor.
**Gate G6.16** — debris strip (8–10 mm dowel/cable taped across the course): **≥ 8/10 crossings
without fall** with reflex on, vs reflex-off baseline recorded (expect snags); **0 false
triggers in 5 min** of flat walking.

## 6. R6 — convex-MPC-lite (stretch; honesty first)
**Literature pointers** (read, don't build, until the gate's decision says otherwise): Di Carlo
et al., *Dynamic Locomotion in the MIT Cheetah 3 through Convex Model-Predictive Control* (IROS
2018); Kim et al., *Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and MPC*
(2019, Mini-Cheetah); MIT's open-source Cheetah-Software and its hobby-grade reimplementations
(verify availability at execution time).
**Why full MPC is unrealistic on BARQ**: convex MPC emits ground-reaction FORCES at 25–50 Hz
consumed by a torque-controlled whole-body layer at 0.5–1 kHz. The ST3215 exposes **position
goals only** (no torque interface), behind USB CDC + a 100 Hz position command stream with
10–30 ms full-chain latency (P3-measured TBD) and a k≈60/s internal servo loop — force tracking
through that chain is fiction. An "MPC-lite" (body-pose QP → foot positions, still
position-interfaced) is the only honest variant, and it is approximately R1+R2+R3 generalized —
for the flat-indoor mission set its marginal gain over rungs 1–5 is small.
**Gate G6.17 (decision gate)** — a go/no-go ADR in `docs/02_DECISIONS.md`: go ONLY with a
written latency/torque-path budget proving the loop closes; default and expected outcome:
**no-go, rungs 1–5 suffice**. Recording the no-go with the budget table IS the pass.

## Acceptance gates (summary)
**G6.12** R1 ≥ 85 % realized @0.10 · **G6.13** R2 |yaw| < 0.05 rad over 5 m at duty 0.6 ·
**G6.14** R3 ±5° ramp, error halved · **G6.15** R4 step-cmd no-lurch + nav2 mission ·
**G6.16** R5 ≥ 8/10 debris crossings, 0 false triggers · **G6.17** R6 ADR recorded.
Protocol §6(c) is satisfied at G6.13; everything after is mission appetite.

## Fallback ladder (per rung — rungs never block each other)
A: tune the rung's single gain, one knob per run → B: halve the gain + strengthen the low-pass
(loops must be boring before they're good) → C: disable the rung (param off) and ship without
it — each rung is independently shippable by construction. Switch after 2 failed sessions on a
rung; a failed rung is a research-log entry, not a phase blocker.

## Rollback
All rungs param-gated, default OFF (`speed_fb`, `yaw_fb`, `pitch_reflex`, `cmd_shaping`,
`stumble_reflex`); rollback = relaunch with the flag off — behavior returns to P4's qualified
open-loop gait. The only code outside the gait node is R5's read-only load topic (passive,
P4-CODE-1 class). Git-revert only for build breakage.

## TBD table
| # | Unknown | Procedure |
|---|---|---|
| 1 | R1 multiplier saturation point (reach vs torque) | climb `speed_fb_max` per run; log realized % + `/barq/diag` loads until flat |
| 2 | R2 residual yaw + final kp | before/after 5 m table at duty 0.55 / 0.6 (closes Q-016) |
| 3 | R3 trim envelope check + measured °/cm on the robot | one-off `ik_exact` extreme check; ramp bags vs rear_raise log |
| 4 | R5 swing-load threshold | §5 bag procedure (floor p99 × 1.3) |
| 5 | R4 ramp limits nav2 tolerates | mission run with ramps on, controller timeouts watched |

## Artifacts → docs/05_RESEARCH_LOG.md
Per rung: before/after measured table (the same metric the rung claims to improve), final gains,
the gate evidence (tape numbers, bags, videos), and a one-line ADR if any default changes.
Q-013's residual and Q-016 get UPDATED in `docs/04_OPEN_QUESTIONS.md` when R1/R2 land — closing
measured questions is the publication record. Commit + push per session.

## Escape hatch
If even rungs 1–2 cannot reach their gates, the bottleneck is BELOW control: estimator drift
out of spec (back to P4-03's fallbacks), calibration (P2 re-zero), or actuation (P3 bench ID) —
fix the layer, not the loop. If the rungs pass and the robot still disappoints, the mission is
mis-scoped, not the ladder: re-scope speeds/terrain in an ADR — the Protocol's finish line needs
rung 2, not rung 6 — and remember the seam to RL stays open: every rung here also makes the
eventual policy's baseline (G6.5/G6.10) harder and more honest, which is exactly what a
publication wants.
