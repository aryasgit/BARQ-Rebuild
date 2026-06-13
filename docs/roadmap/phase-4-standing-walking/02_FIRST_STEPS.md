# P4-02 — First Steps: walking progression + the tuning decision tree

> Phase P4 · verified against repo @ 4ea53a0

## Objective
From standing (G4.1/G4.2) to repeatable walking: in-place weight shift, then a speed
ladder 0.05 → 0.10 → 0.15 m/s and turns, each level passed 3 consecutive runs, measured
with a hardware walk metric (tape measure = truth; the estimator's claim recorded
alongside, which calibrates its drift for free). Exit gates **G4.3** (5 m straight),
**G4.4** (closed circle ×2), **G4.5** (30 min thermal/fault endurance).

## Prerequisites
1. **G4.1, G4.2 passed** (01_FIRST_STAND).
2. **G4.6 and G4.7 passed** (`03_ESTIMATOR_AND_SAFETY.md` §6) — the stop-ladder drill
   and the fall-detect cradle test come BEFORE floor walking. No exceptions.
3. P4-CODE-1 telemetry live (`/barq/diag`, `/imu/data`); P4-CODE-3 (`hw_walk_metric.py`,
   §3 below) built and smoke-tested against the emulator.
4. Course prepared: ≥ 6 m of hard smooth floor (tile/wood), a 5 m tape line on the
   floor, start cross-mark, a 90° protractor-drawn cross at the start (heading datum),
   foam pad at course end. Spotter walks alongside for every NEW level's first run.
5. Battery ≥ 15.5 V resting; master switch carried by the operator (long lead or
   walk-along).

## How to run a level (the standard run)
Fresh stand per run — between-run state is part of the experiment (00 §2): power
state stays up, but bring the robot back to the start cross, let it stand 5 s, then run.

```
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true
ros2 bag record -o walk_$(date +%H%M) /barq/diag /joint_states /imu/data /odom_est /cmd_vel
python3 /home/barq/barq_ws/src/diagnostics/hw_walk_metric.py --vx 0.05 --duration 60
```
`hw_walk_metric.py` streams `/cmd_vel` continuously (satisfies the gait node's 1.0 s
deadman; `teleop_twist_keyboard` only publishes on keypress — use it for free driving,
never for measured runs). Choose `--duration` from the EXPECTED ~60 % realization:
distance ≈ 0.6 × vx × duration (0.05 → 60 s ≈ 1.8 m; line targets below).

Non-default gait params: `real.launch.py` exposes only `device`/`gait` — for tuned runs
use `gait:=false` plus manual nodes:
```
ros2 run barq_control ik_node
ros2 run barq_control gait_planner --ros-args -p duty:=0.55 -p period:=0.6
```
*(Optional 6-line launch addition: mirror `sim.launch.py`'s `gait_duty`/`gait_period`
DeclareLaunchArgument + parameters dict into `real.launch.py` — same names, default
0.6/0.5. Do it when manual nodes get tedious; not required.)*

## Procedure

### 1. Step 0 — in-place weight-shift ("march")
The current gait has no true march: at zero `/cmd_vel` feet hold stance (no stepping).
Workaround that needs zero code: command a crawl that produces ~6 mm steps —
`hw_walk_metric.py --vx 0.02 --duration 20` (step length = vx·period·duty =
0.02·0.5·0.6 = 6 mm). Watch for: rhythmic diagonal weight transfer, all four feet
actually breaking contact, no foot dragging a groove, body sway < ~2 cm, no fall.
3 clean repetitions → proceed.

**P4-CODE-4 (optional, ~20 lines) — a real march mode:**
- `barq_control/barq_control/gait.py` — `foot_targets(..., march=False)`: change the
  gate to `moving = march or (abs(vx)+abs(vy)+abs(wz)) > deadband`, and when `march`
  zero the step vector (`sx = sy = 0.0` unless moving) so the cycle lifts in place
  (~6 lines).
- `gait_planner_node.py` — `declare_parameter('march', False)`, pass through to
  `foot_targets`; march steps only while `/cmd_vel` messages are ARRIVING (any value,
  even zero twist) so the 1.0 s deadman chain still stops everything on teleop death
  (~8 lines: track `last_cmd_time` non-None and fresh).
- Unit test: march + zero cmd ⇒ feet z cycles, x/y constant (~6 lines).

### 2. The speed ladder (3 consecutive clean runs to pass a level)
| Level | cmd | course target (tape) | duration | pass bar per run |
|---|---|---|---|---|
| L1 | vx 0.05 | 3 m line | ~100 s | finishes ≥ 2.4 m, \|lat\| < 0.4 m, no fall, no assist |
| L2 | vx 0.10 | 5 m line | ~85 s | finishes ≥ 4.0 m, \|lat\| < 0.5 m, no fall, no assist |
| L3 | vx 0.15 | 5 m line | ~60 s | finishes ≥ 4.0 m, \|lat\| < 0.7 m, no fall, no assist |
| LT | vx 0.10 + wz 0.3 | circle R = v/ω ≈ 0.33 m | ≥ 1 lap | closes the loop within 0.5 m of start (sim twin: clean R≈0.4 circle at 0.12/0.3) |

A fall, an assist (hand touch), or a fault bit = failed run. 3 fails at a level →
tuning tree (§4), restart the level count after any knob change.

### 3. The HARDWARE walk metric (tape is truth; estimator graded alongside)
There is no `/odom_gt` on hardware. Per run record:
- **tape_fwd**: start cross → final position of a fixed body datum (plumb the FRONT
  hip axle centre to the floor with a weighted string, mark, measure along the line).
- **tape_lat**: perpendicular offset of that mark from the line.
- **heading**: angle of the body's final centerline vs the floor tape — lay a straight
  edge along the body axis, mark two points, measure with the protractor cross
  (± 2° achievable). A phone compass on the deck is the backup (± 5°, keep it away
  from the servos' magnets).
- **estimator claim**: `/odom_est` displacement over the same window — printed by the
  tool below. est_vs_tape error % = |est_fwd − tape_fwd| / tape_fwd × 100. This single
  number, collected on every run, IS the P4-03 drift dataset — file it there too.

**P4-CODE-3 (required tooling, diagnostics/hw_walk_metric.py, ~30 lines diff from
sim_walk_metric.py):** copy `sim_walk_metric.py`, then: subscribe `/odom_est` instead
of `/odom_gt`; DELETE the `use_sim_time` parameter line (wall clock); add `--wz`
argument fed to `cmd.angular.z`; after the run `input()` three prompts (tape_fwd,
tape_lat, heading_deg) and print one parseable line:
`HW_WALK vx=… wz=… T=… est_fwd/lat/yaw=… tape_fwd/lat/yaw=… est_err=…% realized=…%`.
Same settle/stop structure; keep the 5× zero-Twist stop burst.

### 4. Tuning decision tree (ORDER MATTERS — one knob per run, log every run)
Apply the FIRST matching symptom only; re-run the level; never stack two changes.

1. **Falls / instability** → `period` +0.1 (0.5 → 0.6, cap 0.7) — slower cadence
   FIRST, before touching anything else. (Note step length = vx·period·duty grows
   with period; that is fine, cadence is the stabilizer.)
2. **Veer / curved track** → `duty:=0.55` (Q-016: sim zero-crossing — dead-straight
   but slower, ~47 % vs ~60 % realized; that trade is the deal). If straightness at
   0.55 still fails: the proper fix is estimator-yaw feedback into `wz`
   (P6-05 rung 2, ~20 lines) — preview it only if G4.3 is blocked, and measure
   before/after for the log.
3. **Speed deficit** → realized ~60 % of commanded is **NORMAL** (Q-013 mechanism:
   swing-foot drag at the ~20 mm front-clearance ceiling; μ-invariant, proven in sim).
   Do NOT chase 100 % open-loop — that is what closed-loop/RL (P6) is for. Investigate
   ONLY if realized < 45 % (below the sim band): check temps, vbus sag during stride,
   zero offsets, and §5 surface.
4. **Toe-stubs on the real floor** (audible scuff / tripping mid-swing) →
   `step_height` +0.005 ONLY with `stand_height` +0.005 PAIRED (one paired change =
   one knob). Reach ceiling math: **stand − step ≥ 0.110 m** (absolute in-plane fold
   floor 0.1079 at tibia −2.2; 0.110 keeps 2 mm margin). 0.135/0.025 → 0.140/0.030
   max, then stop (taller stand = higher CoM); fall back to rung 1/2 thinking.

### 5. Surface ladder
Hard smooth floor (all gates) → short carpet → outdoor flat concrete. One L2-style run
per new surface before driving freely. Expect: carpet swallows clearance (rung-4
symptoms) and changes estimator slip — record est_vs_tape on EVERY surface (P4-03
needs both numbers). Sim says realized-speed is μ-invariant — if a surface changes
realized speed a lot, suspect geometry (pile height), not friction, and say so in the log.

### 6. Endurance (G4.5)
One session: accumulate 30 min of total walk time (sum of run durations, torque on
throughout; pauses standing are allowed and counted) on L1/L2 mixes. Log temp_max
every 5 min; abort at 65 °C (the P4-03 watchdog bar), record the curve, plus vbus
start/end (sag feeds P1 table).

## Regression table (template — fill per level, keep in the research log)
| run | surface | vx_cmd | wz_cmd | realized m/s (tape) | realized % | \|lat\| m / 5 m | heading drift ° | est_vs_tape % | temp_max °C | vbus sag V | knobs (period/duty/step/stand) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| sim baseline | gz flat | 0.15 | 0 | 0.090 | **~60 %** | 0.0004/10 s | +0.3 rad/10 s left | n/a (gt) | n/a | n/a | 0.5/0.6/0.02/0.13 |
| sim straight | gz flat | 0.15 | 0 | ~0.071 | ~47 % | — | −0.04 rad/10 s | n/a | n/a | n/a | 0.5/**0.55**/0.02/0.13 |
|  |  |  |  |  |  |  |  |  |  |  |  |

The sim rows are the comparison column: hardware realized % within ~±15 points of the
sim row at the same knobs = the gait TRANSFERRED; bigger gaps → tree §4, and write the
delta up (that comparison is publication material).

## Acceptance gates
- **G4.3 — 5 m straight**: vx 0.10, 5 m course, |lateral| < 0.5 m at the end,
  unassisted, no falls/faults — **×3 consecutive**.
- **G4.4 — closed circle**: vx 0.10 + wz 0.3, ≥ 1 full lap returning within 0.5 m of
  the start mark — **×2** (once each direction if both pass; log both regardless).
- **G4.5 — endurance**: 30 min cumulative walk time in one session, temp_max never
  > 65 °C, zero fault bits, vbus ≥ 13.6 V throughout.

## Fallback ladder
- **A — level fails ×3**: tuning tree §4 (one knob), restart level count.
  Switch: 3 knob-runs without improvement.
- **B — drop one level**, re-pass it ×3, climb again. Switch: 2 climb failures.
- **C — yaw authority absent in LT** (no turning / turns the wrong way): check the
  coxa direction multipliers (FR/RR = −1 in robot_params.yaml) and coxa zero offsets
  on the bench; re-run the P3 air-walk yaw check on the cradle (wheels-up turn motion
  visible). If still dead → park, `docs/04_OPEN_QUESTIONS.md`, Escape hatch.

## Rollback
Gait params to defaults (0.5/0.6/0.02/0.13, rear_raise 0.02): kill manual nodes,
relaunch with `gait:=true`. A paired step/stand change reverts as a pair. Robot to
cradle; verify a 10 s stance hold before ending the session.

## TBD table
| # | Unknown | Procedure |
|---|---|---|
| 1 | Hardware realized-% band per level (sim says ~60 % @0.15/duty 0.6) | §2 ladder, 3 runs/level → regression table |
| 2 | Carpet/outdoor deltas (realized %, est_vs_tape %) | §5 one run per surface |
| 3 | Sustained walking current draw (watchdog bar input) | §6 endurance bag, mean/p95 of `/barq/diag`[13] → P4-03 TBD 1 |
| 4 | Does duty 0.55 stay dead-straight on hardware (Q-016 transfer)? | tree rung 2 A/B runs, same course |

## Artifacts → docs/05_RESEARCH_LOG.md
Filled regression-table rows (every run, including fails); HW_WALK lines; bag files;
knob-change log (one line per change: symptom → knob → delta); G4.5 temp curve.
Update `docs/01_STATUS.md` one-liner; commit + push (00 §1.5).

## Escape hatch
Blocked ≥ 2 sessions below G4.3: re-prove G4.1 (stand regressions hide here), re-run
the P3 air-walk metric for tracking drift, then park with full numbers in
`docs/04_OPEN_QUESTIONS.md` and advance P5-01 or P6-01/02 (00 §5). A robot that
stands + marches but won't pass L2 is still a valid platform for P4-03's estimator
drills at L1 speeds — do those in parallel rather than stalling the phase.
