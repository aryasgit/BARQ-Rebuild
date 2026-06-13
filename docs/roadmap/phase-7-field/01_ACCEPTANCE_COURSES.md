# P7-01 — Acceptance Courses: graduated physical trials with sim-bound regression tables

> Phase P7 · verified against repo @ 0e5ddaf

## Objective
Prove the robot in the real world on five graduated courses — from a tape line in a hallway to a
full physical replica of the sim obstacle course (`barq_sim/worlds/barq_course.sdf`, the 2d
research-log run) — and quantify the sim-vs-real gap per course in a regression table. Course 3
is the headline: the same 1 m doorway → 3-pillar slalom → box field, one NavigateThroughPoses
mission, ~16 m, that the sim completed with 0.157 m final error and autonomous spin/wait
recoveries. Pass all gates G7.1–G7.4 with the classical stack; G7.5 repeats under the RL policy
(or is skipped via the P6-05 bypass ladder, recorded as such).

## Prerequisites
- [ ] P4 gates closed: untethered walk, straight 5 m repeatable.
- [ ] P5 gates closed: lidar integrated, SLAM + nav2 on robot, at least one indoor A→B mission.
- [ ] `02_OPERATIONS_RUNBOOK.md` PRE-RUN checklist passing (use it for every session here).
- [ ] Mission discipline from sim 2d applies verbatim: **no robot-side GUIs during missions;
      verify by telemetry, never by a client's exit code** (sim build #4 / lost-goal lessons).
- [ ] Hardware nav speed ceiling starts at **0.10 m/s** (P5 value; sim's 0.22 m/s ceiling is
      raised only after G7.2, one increment at a time, max +0.04 m/s per session).
- [ ] Materials: masking tape, chalk, 5 m+ tape measure, phone (stopwatch + video), 4–6 cardboard
      boxes ≥ 0.3 m tall, 3 chairs (or buckets) + cardboard wrap sheets, paint pen.
- [ ] Battery resting ≥ 15.4 V at session start; abort floor 13.6 V (P1 rules).

### Why obstacle heights matter (read once)
The lidar scan plane sits ≈ 0.21 m above the floor (P5 mount, −4.5° counter-wedge cancelling the
D-016 stance pitch). **Anything shorter than ~0.25 m is invisible to the costmap.** Every
obstacle below must present a surface ≥ 0.30 m tall at the scan plane. Chair legs are too thin
to mark reliably — wrap each chair's leg cluster into one cardboard cylinder/box ≥ 0.15 m wide.
Corridor floor width ≥ 1.0 m everywhere (robot_radius 0.18 + inflation 0.30 in
`barq_bringup/config/barq_nav2.yaml`; same arithmetic as the sim course comment).

## The standard bag set (record EVERY run, all courses)
Logging is on by default (runbook RUN step 6). One command, size-capped:

```
ros2 bag record -o ~/barq_ws/bags/$(date +%Y%m%d_%H%M%S)_courseN \
  --max-bag-size 1073741824 \
  /joint_states /imu/data /odom_est /cmd_vel /foot_targets /tf /tf_static \
  /scan /map /plan
```

- Courses 1 and 4 (no autonomy): drop `/scan /map /plan` if lidar is not running.
- The C++ `ros2 bag record` is the trusted recorder (rclpy live sampling lies on a loaded
  Jetson — 2f lesson). Analyze offline with `diagnostics/analyze_track_bag.py`.
- After every run: check the bag closed cleanly (`ros2 bag info <dir>`) before moving anything.

| TBD | How to fill |
|---|---|
| Telemetry topic name for vbus/current/temp_max/fault (STATE frame fields) | P3/P4 exporter decides (expected `/barq/telemetry`); add it to the bag command above and to the runbook once it exists |
| Hidden nav2 action feedback topics worth bagging | try `--include-hidden-topics` on one C2 run; keep only if bag size stays < 2 GiB |

## Measurement protocol (common to all courses)
1. Fresh power-cycle per measured run (between-run state is part of the experiment — sim lesson).
2. Tape a start line; place the robot's nose on it, feet symmetric about the course axis.
3. Phone video from the side for the whole run (frame the start line); stopwatch from the
   stand-complete moment to mission end.
4. Final position error: tape-measure from the goal mark on the floor to the mid-point between
   the two front feet. Write it down before touching the robot.
5. est-vs-tape: from the bag, integrate `/odom_est` path length; compare to the taped course
   length. Report as % of distance (sim baseline: 4–5%).
6. A run "counts" only if started from the runbook checklist and bagged. No bag → no run.

---

## Course 1 — flat 10 m hallway line (open-loop gait, no nav2)

### Build
```
 START                                                 FINISH
   |  1m   2m   3m   4m   5m   6m   7m   8m   9m  10m   |
 ──┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼──
   |==== chalk / tape centerline, marker every 1 m =====|
   (hallway ≥ 1.5 m wide, hard flat floor, cleared)
```
- Tape a straight 10.00 m centerline (verify with tape measure), cross-marks every 1 m.
- No obstacles, lidar optional (bag set without `/scan /map /plan` is fine).

### Procedure
1. Runbook PRE-RUN + RUN through the stand gate. Start the bag.
2. Command straight walk at the P4-proven speed (start 0.10 m/s):
   `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.10}}"`
3. Walk the full 10 m; kill the publisher (gait deadman zeros the command in 1 s).
4. MEASURE per run: (a) time start-line → 10 m line; (b) lateral deviation of the body
   centerline from the tape at 5 m and at 10 m (tape measure, sign = left+); (c) est-vs-tape.
5. Three runs, fresh power-cycle each. Record all three rows even if one fails.

### Regression table (fill all rows; sim numbers are the bind)
| Metric | Sim baseline | Run 1 | Run 2 | Run 3 | Real/sim gap |
|---|---|---|---|---|---|
| Realized/commanded speed | ~60% (duty 0.6, 2f) | | | | |
| Lateral dev @ 10 m | ~0 (D-016 trim, scaled) | | | | |
| est-vs-tape (% of dist) | 4–5% (2e) | | | | |
| Falls / human touches | 0 | | | | |

### Gate G7.1
- [ ] 3/3 runs complete 10 m with zero falls and zero human touches.
- [ ] Lateral deviation at 10 m ≤ 0.5 m on every run (5% of distance).
- [ ] est-vs-tape ≤ 10% of distance on every run (kinematic-odometry band).
- [ ] Realized speed ≥ 40% of commanded (if below: gait transfer regressed — back to P4 tuning).

### Fallback ladder (G7.1)
- **A.** Fails on lateral drift → re-run P4 straightness trim (duty/rear_raise sweep on hardware,
  one parameter at a time, 3 runs each). Switch to B after 2 sessions without passing.
- **B.** Fails on est-vs-tape → P4 estimator re-check at stance + slow walk; verify stance-diagonal
  selection against the real gait (the 2e D-016 bug class). Switch to C after 1 session.
- **C.** Fails on falls/speed → drop cmd to 0.08 m/s and re-attempt; if still failing, P7 is
  premature — reopen P4's last gate and do not return until it re-passes.

---

## Course 2 — home slalom (3 chairs, autonomous nav2 mission)

Mirrors the sim pillar slalom (pillars r=0.2 at staggered ±0.9 m lateral) with a relaxed 1.5 m
longitudinal spacing — the on-ramp to Course 3's exact geometry.

### Build
```
            chair A (wrapped)        chair C (wrapped)
                [A]                      [C]
 START ──►            ~1.5 m   [B]   ~1.5 m            GOAL mark
  (0,0)                     chair B                    (~6 m out)
        lateral stagger: A left ~0.7 m, B right ~0.7 m, C left ~0.7 m
        floor gap past each chair ≥ 1.0 m
```
- Wrap each chair's legs into one cardboard column ≥ 0.15 m wide, ≥ 0.30 m tall.
- Tape an X at the goal, ~6 m from start, beyond chair C. Measure and note all positions.

### Procedure
1. Runbook PRE-RUN + RUN: full autonomy stack (real.launch + lidar + SLAM + nav2, per P5).
   No RViz on the robot; if you must watch, RViz on the Mac over the network (P5 viz rules).
2. Start the bag (full set). Confirm `/map` is updating and `/scan` ≈ 10 Hz before sending.
3. Send NavigateToPose to the goal X (P5 mission client; see runbook RUN step 7 for the
   send-and-verify pattern — telemetry, not exit codes).
4. Hands off. Count autonomous recoveries (spin/wait/backup in the behavior_server log).
5. MEASURE: time, final error (tape to X), recoveries, chair contacts (video review).
6. Three runs, fresh power-cycle each.

### Regression table
| Metric | Sim baseline | Run 1 | Run 2 | Run 3 | Gap |
|---|---|---|---|---|---|
| Mission result | SUCCEEDED (2c: 0.14 m err over 2.6 m) | | | | |
| Final error (tape) | 0.14 m (2c, shorter mission) | | | | |
| Autonomous recoveries | 0 (2c) / spin+wait seen in 2d | | | | |
| Obstacle contacts | 0 | | | | |

### Gate G7.2
- [ ] ≥ 2 of 3 runs: SUCCEEDED, zero chair contact, zero human touches.
- [ ] Final error < 0.3 m on the passing runs.
- [ ] ≤ 2 autonomous recoveries per passing run.

### Fallback ladder (G7.2)
- **A.** Goal rejected / robot won't move → runbook integration ping, then P5 failure tree
  (action server up? map→odom alive?). 2 failed attempts → power-cycle everything once.
- **B.** Repeated stalls/recoveries near chairs → wrap columns wider (0.25 m), re-run; if still
  stalling, widen gaps to 1.2 m and re-run; if that passes, the real footprint+inflation margin
  is thinner than sim — note it, then tighten back stepwise.
- **C.** Robot clips obstacles → raise `inflation_radius` 0.30 → 0.35 in the **hardware** nav2
  yaml (never edit the sim baseline), re-run from step 2. Record the divergence in the table.

---

## Course 3 — the sim-course replica (doorway → slalom → box field, 2-waypoint mission)

### Build (10 × 8 m if available; see fallback for smaller homes)
Sim geometry to replicate (`barq_course.sdf`, coordinates in m from course center):
```
   y↑                outer boundary 10 × 8 m (walls/furniture/tape)
    │ [box -3.4,1.6]                         │door │
 WP2✕                                        │ gap │        (s1)2.7,0.9
    │        [box -1.6,0.9]                  │ 1.0 │              (s3)4.2,0.7
    │  [box -2.8,-0.6]          START(0,0)   │  m  │   (s2)3.5,-0.9      WP1✕
    │       [box -2.0,-2.2]                  │wall x=1.5│
    └────────────────────────────────────────────────────────────────► x
      box field (west)            doorway          slalom (east)
```
Build list (cardboard, ₹0 if scavenged):
- [ ] Doorway: a wall line at x = +1.5 m with a 1.0 m gap centered on the course axis. Use a real
      door frame if the room offers one, else two rows of boxes ≥ 0.3 m tall as wall segments.
- [ ] Slalom pillars at (2.7, +0.9), (3.5, −0.9), (4.2, +0.7): wrapped chairs or buckets,
      ≥ 0.15 m wide, ≥ 0.3 m tall (sim pillars r = 0.2 m).
- [ ] Box field at (−1.6, +0.9), (−2.8, −0.6), (−2.0, −2.2), (−3.4, +1.6): cardboard boxes,
      footprint 0.3–0.7 m, height ≥ 0.3 m, weighted (a book inside) so a brush doesn't move them.
- [ ] Waypoint marks: WP1 tape-X east beyond the slalom (~4.5, 0); WP2 tape-X at (−3.6, +2.4) —
      the sim's WP2. Tape the start at (0,0). Measure every obstacle position with the tape and
      write the as-built coordinates next to the table below (the map is the truth, not the plan).
- [ ] Walk the course yourself: every gap the robot must thread ≥ 1.0 m of floor.

### Procedure
1. Runbook PRE-RUN + RUN: full autonomy stack, fresh SLAM map (unknown course — that is the
   point; do not pre-map). Bag on (full set).
2. Confirm telemetry before goal-send: `/scan` ≈ 10 Hz, `/map` growing, `/odom_est` quiet at
   stance.
3. Send ONE NavigateThroughPoses mission with 2 waypoints (WP1 then WP2), exactly like the sim
   2d run (~16 m path). Use the P5 mission client / runbook RUN step 7 pattern.
4. Hands off, completely. Spot only for a genuine fall. Log recoveries from behavior_server.
5. At mission end: photo of final pose, tape-measure final error to WP2's X, stop the bag.
6. MEASURE: completion, time, final error, recovery count+types, contacts, est-vs-tape over
   the full path, max servo temp (POST-RUN temp note).
7. Two runs minimum, fresh power-cycle + course reset (boxes back on their tape marks) between.

### Regression table — THE sim-vs-real instrument (one row per run, keep forever)
| Metric | Sim 2d baseline | Real run 1 | Real run 2 | Gap / notes |
|---|---|---|---|---|
| Mission completed | YES (one NTP cmd, 2 WPs, ~16 m) | | | |
| Final error vs WP2 | **0.157 m** ((−3.48,2.30) vs (−3.6,2.4)) | | | |
| Recoveries (type) | spin + wait, self-recovered | | | |
| Human touches | 0 | | | |
| Mission time | record from sim bag if kept; else "—" | | | |
| est-vs-tape (% dist) | 4–5% (2e, shorter path) | | | |
| Obstacle contacts | 0 | | | |
| Speed ceiling used | 0.22 m/s (sim RPP) | 0.10 m/s start | | expected slower — note ratio |

### Gate G7.3 (the phase headline)
- [ ] Mission completes (both waypoints) on ≥ 2 runs.
- [ ] Final error < 0.3 m (sim did 0.157 m; <0.3 m allows the hardware estimator band).
- [ ] ≤ 2 autonomous recoveries per run, all self-resolved.
- [ ] Zero human touches, zero relocations of the robot.
- [ ] Regression table filled and committed; gaps vs sim explained in one sentence each.

### Fallback ladder (G7.3)
- **A.** Mission stalls mid-course (> 2 recoveries or progress-checker abort) → re-run once after
  full power-cycle. Still failing → split the mission: run doorway+slalom as one NavigateToPose,
  box field as another. Pass both → the deficit is mission length (battery sag / map drift):
  check vbus trend in the bag, then retry the full mission with a fresh-charged pack.
- **B.** Robot loses localization (map jumps, plan oscillates) → slow down: drop the RPP ceiling
  one notch; add wall texture (boxes along bland walls — sim 2b: featureless = no map). 2 failed
  sessions → reopen P5 SLAM tuning, P7 paused.
- **C.** Persistent collision at the doorway → verify door gap is a true 1.0 m as built; raise
  inflation 0.05 m at a time (hardware yaml only) until clean, then record the final value as the
  real robot's corridor constant. If inflation > 0.40 still clips: footprint reality differs from
  the 0.18 m model — re-measure the robot's true swept width and update the hardware yaml.
- **D.** Course physically too large for the home → scale to 8 × 6 m, SAME topology (doorway gap
  stays 1.0 m, gaps ≥ 1.0 m). Mark the table "SCALED 0.8×" — sim comparison still valid for
  completion/recoveries/final-error, not for time.

---

## Course 4 — surface ladder (carpet → threshold lip → gentle ramp)

### Build
```
 (a) CARPET 3 m strip      (b) THRESHOLD: door sill / batten, 1–2 cm lip
 ──[==carpet==]──          ──────┐_┌──────   tape 1 m approach + 1 m exit
                                lip ≤ 2 cm
 (c) RAMP ≤ 5° (only if available — plywood on a book stack, ~9 cm rise over 1 m = 5°)
 ──────/▔▔▔▔▔\──────  walk up, across, down; ramp ≥ 0.8 m wide
```

### Procedure (per surface: carpet, threshold, ramp)
1. Runbook PRE-RUN + RUN through stand gate. Bag on (gait set). Open-loop straight walk
   (Course-1 style) at 0.08 m/s — softer than Course 1; surfaces eat clearance margin
   (sim 2f: swing clearance ceiling ~20 mm; a 2 cm lip is exactly that budget).
2. Carpet: 3 m line on the carpet, Course-1 metrics (time, lateral, est-vs-tape).
3. Threshold: approach perpendicular, 3 crossings; count foot snags (video frame-by-frame).
4. Ramp (if available): up, hold 3 s, across, down. Spotter hand NEAR (not touching) on first
   attempt — new-terrain rule from the Doomsday Protocol cradle ladder.
5. Two clean runs per surface; fresh power-cycle between surfaces, not between runs.

### Regression table
| Surface | Sim analogue | Completes | Falls | Snags | est-vs-tape | Notes |
|---|---|---|---|---|---|---|
| Carpet 3 m | none (sim μ-invariant, 2f — predicts mild slowdown only) | | | | | |
| Lip 1–2 cm | none — at the ~20 mm swing-clearance ceiling | | | | | |
| Ramp ≤ 5° | none (flat-ground sim) | | | | | |

### Gate G7.4
- [ ] Carpet: 2/2 runs complete 3 m, no falls.
- [ ] Threshold: ≥ 2 of 3 crossings clean (≤ 1 snag total, no falls).
- [ ] Ramp (if built): up-across-down without fall, 2/2. If no ramp available, mark N/A —
      the gate passes on carpet + threshold alone (record "ramp untested" in the log).

### Fallback ladder (G7.4)
- **A.** Threshold snags → raise `step_height` one notch on hardware params, re-run; watch the
  tibia envelope (stand − step ≥ 0.095 m constraint in `gait_planner_node.py`). 2 failures → B.
- **B.** Still snagging → approach at 45° (one foot pair at a time crosses the lip), re-run.
  Works → record as an operational constraint ("lips taken oblique"), gate passes with note.
- **C.** Carpet fails (stumbles, yaw blow-up) → drop to 0.06 m/s; if still failing, carpet is
  out-of-envelope for the classical gait — record it, gate this surface N/A, raise it as a P6
  RL target (terrain randomization), continue P7 on hard floor only.

---

## Course 5 — policy era (after P6): repeat 1–3 under RL + push-recovery demo

Only entered when P6 deployed a policy (if P6 closed via the no-RL bypass, mark G7.5
"BYPASSED per P6-05" and the phase ends at G7.4 — that is a legitimate finish).

### Procedure
1. Re-run Courses 1, 2, 3 under the policy, identical builds, same bag set, same tables —
   append a "policy" column beside the classical results. New gait on hardware = cradle first,
   then floor with spotter, then free (Protocol §3), even though it passed in sim.
2. Push-recovery demo (the policy's signature trick): robot standing under policy; calibrated
   nudge at the shoulder — start with a 0.5 kg object pendulum-swung from 20 cm, escalate only
   if trivially recovered. Spotter, soft floor mat, fresh battery. 5 pushes, ≥ 4 recovered
   without a step beyond 0.3 m displacement. Video each.
3. Fill the P6-04 regression comparison: policy vs classical per course (speed, error,
   recoveries, falls).

### Gate G7.5
- [ ] Policy ≥ classical on Course 1 speed AND ≤ classical on Course 3 recoveries (the P6-04
      bar restated in field terms), no new fall modes.
- [ ] Push-recovery: ≥ 4/5 recovered, zero falls.
- [ ] OR: "BYPASSED per P6-05" recorded with the bypass rung reached.

### Fallback ladder (G7.5)
- **A.** Policy worse on one course → keep classical as the deployed default, file the gap as a
  P6 retraining item (domain randomization target = the failing surface/obstacle), re-attempt
  after one retrain cycle. The robot stays field-operational on classical meanwhile.
- **B.** Policy unsafe (new fall mode) → torque-off, classical stack restored the same session
  (rollback below), policy quarantined to cradle-only until P6 sim2real review.

---

## Rollback
Courses change no code. If any course session ends in a degraded robot or a confused stack:
1. Restore params: `git checkout -- barq_bringup/config/` (hardware-yaml experiments revert;
   committed sim baselines were never touched).
2. Policy era: relaunch with the classical gait (`real.launch.py gait:=true`, policy node OFF).
3. Robot damage → runbook CRASH RESPONSE tree, then re-enter at the last passed course, one
   course down from where it broke (pass C3 crash → re-verify C2 before retrying C3).

## Artifacts → docs/05_RESEARCH_LOG.md
Per course (same session, before charging the battery — runbook POST-RUN step 5):
- The filled regression table row(s) — believed (sim baseline) / measured (real) / gap / why.
- Bag directory names, photos of as-built course with tape measurements, video filenames.
- Any hardware-yaml divergence from sim values (inflation, speed ceiling) as an override entry.
- Gate verdict line: `G7.x PASS/FAIL — <one sentence>`. Also tick `appendices/B_ACCEPTANCE_GATES.md`.

## Escape hatch
If, after the fallback ladders, no course beyond G7.1 can be passed in 3 consecutive sessions:
stop building courses. The robot you have IS the deliverable — a 10 m-capable walker with the
full sim-proven stack. Write the sim-vs-real gap analysis from whatever tables are filled (that
analysis is publication material on its own), file the blocking symptom in
`docs/04_OPEN_QUESTIONS.md` with all measurements, and run the robot inside its proven envelope
using `02_OPERATIONS_RUNBOOK.md`. A smaller honest envelope beats a broken robot chasing a gate.
