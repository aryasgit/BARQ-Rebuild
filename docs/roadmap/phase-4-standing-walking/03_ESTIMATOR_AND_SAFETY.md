# P4-03 — Estimator on Hardware + Safety Systems

> Phase P4 · verified against repo @ 4ea53a0

## Objective
(1) Qualify the v1 legged odometry on real metal — drift vs tape truth on an L-course,
IMU yaw sanity, slip sensitivity across surfaces. (2) Stand up the phase's safety
systems: the software stop ladder AS IT ACTUALLY IS in the code (documented + drilled),
param-gated fall detection, and the telemetry/watchdog path — including **P4-CODE-1,
the one REQUIRED hardware-interface code addition of this phase**. Exit gates
**G4.6** (stop drill), **G4.7** (fall-detect), **G4.8** (battery-floor drill).

**Execution order within P4**: §5 (code) is a prerequisite of 01_FIRST_STAND; §6
drills G4.6/G4.7 run immediately after G4.1, BEFORE any floor walking in 02; §1–§4
(estimator) need 02's L1 level (and improve with G4.3). G4.8 runs whenever the pack
is naturally near 13.7 V.

## Prerequisites
1. For §5 specs: repo builds clean (`colcon build`), emulator available
   (`ros2 run barq_hw teensy_emulator`).
2. For §6 drills: G4.1 passed; cradle; foam pad; spotter; phone slow-mo (240 fps).
3. For §1–4: 02's L1 passed; the taped L-course (below); `hw_walk_metric.py`
   (P4-CODE-3) in use — every 02 run already produced an `est_vs_tape` number; bring
   that list.

## Procedure

### 1. Estimator bring-up on hardware
`state_estimator_node` consumes `/imu/data` + `/joint_states`. On hardware NOTHING
publishes `/imu/data` until P4-CODE-1 (§5a) is in — the IMU bridge is part of that
one addition (the firmware already streams quat/gyro/accel in every STATE frame;
they just never left the hardware interface).
```
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true
ros2 run barq_control state_estimator        # publish_tf stays false until P5 needs TF
ros2 topic echo /odom_est --once
```
Sanity: robot standing still → `/odom_est` x/y wander < 1 cm/min; carry-walk the robot
by hand (torque off, joints back-drivable… no: joint states freeze without motion) —
skip; the real checks are §2/§3.

### 2. IMU yaw sanity vs floor-tape angles
On the protractor cross from 02: robot standing at 0°. Read yaw
(`ros2 topic echo /odom_est | grep -A4 orientation`, or convert `/imu/data`). Two
hands on the BODY (clear of legs), lift 5 cm, rotate to +90°, set down, settle 5 s,
read again. Repeat 180°, 270°, back to 0°.
- Bar: each 90° step reads 90° ± 5°, sign POSITIVE for counter-clockwise (REP-103).
- Static wander: 10 min stand, log yaw each minute. Bar: < 2° total. If it wanders
  or steps: the BNO085 SH-2 report type is the suspect (mag-fused rotation vector
  jumps indoors) — TBD row 2, fix in firmware (P3 IMU stub) by selecting the game
  rotation vector, re-test.
- Tilt sign check (doubles as G4.7 prep): on the CRADLE, tip nose down by hand →
  pitch must read positive-down consistently with §01 §7's tape-measured 4.5°; roll
  left-up → check sign. Wrong signs = mount-axes mapping error → fix the firmware
  quat mapping BEFORE trusting any estimator number.

### 3. Drift protocol (the v1 qualification number)
Course: tape an L — **3 m straight, 90° left turn, 1 m leg** (4 m path). Origin cross
at start; x along the first leg, y left.
Per run: stand at origin (front-hip plumb mark over the cross, body along x), then
drive the L with teleop at L1–L2 speed (turn in place at the corner with wz only).
At the end: freeze (zero cmd), then
- tape-measure the body datum's final (x, y) in course frame + body heading vs tape;
- record `/odom_est` final (x, y, yaw);
- error = √(Δx² + Δy²) / 4.0 m × 100 %; yaw error in deg.

**Bar (v1): position error < 10 % of distance, |yaw error| < 10°** — deliberately
relaxed from the sim's 4–5 % (hardware adds slip, IMU mounting, real contact timing).
×3 runs on hard floor. Then ×3 on carpet (§4). Log every run; the 02 per-run
`est_vs_tape` percentages are supporting evidence of the same quantity.

### 4. Slip sensitivity note (record, don't fix)
Kinematic legged odometry counts stance-foot motion as body motion — slip inflates or
deflates it. Expect hard floor ≈ best case; carpet different (pile shear). Record
both numbers side by side in the research log. v1 has NO slip rejection by design
(D-017); if carpet exceeds ~15 %, write the delta into `docs/04_OPEN_QUESTIONS.md`
as the v2 motivation (slip gating / accel fusion), do not patch ad hoc in P4.

### 5. SAFETY SYSTEMS — the code, spec'd precisely

#### 5a. P4-CODE-1 (REQUIRED) — `barq_hw` telemetry publishers
**Why**: STATE frames already carry per-servo load, vbus, current, temp_max, fault
(`docs/06_PROTOCOL.md`), but `BarqSystem` only exports pos/vel to ros2_control — the
safety-critical fields die inside the plugin. A second process tapping the serial
port is FORBIDDEN (the port is exclusively owned; opening it twice corrupts framing).
So: extend `BarqSystem` itself. This is the only P4 change at the hardware-interface
layer.

Spec (~55 lines total):
- `barq_hw/include/barq_hw/barq_system.hpp`: members
  `double load_pct_[12]; double vbus_{0}, current_{0}; int temp_max_{0};`
  `rclcpp::Node::SharedPtr tele_node_;`
  `rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diag_pub_;`
  `rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_; int64_t last_diag_us_{0};`
- `src/barq_system.cpp` `on_configure()`: `tele_node_ = rclcpp::Node::make_shared("barq_hw_telemetry");`
  create the two publishers (`/barq/diag` depth 10; `/imu/data` depth 50). Publish-only
  ⇒ no executor/spin thread needed.
- `drain_rx_()` (inside the existing STATE branch): copy
  `load_pct_[k] = s.load_0p1pct[k] / 10.0;` `vbus_ = s.vbus_mv / 1000.0;`
  `current_ = s.current_ma / 1000.0;` `temp_max_ = s.temp_max_c;` and publish one
  `sensor_msgs/Imu` per STATE (100 Hz): orientation = `quat_1e4[0..3]/1e4` (x,y,z,w),
  `angular_velocity = gyro_mrad_s/1e3`, `linear_acceleration = accel_cm_s2/1e2`,
  `header.frame_id = "base_link"` (axes mapping verified in §2), stamp `tele_node_->now()`.
- `read()`: once per second (`mono_us() - last_diag_us_ > 1'000'000`) publish
  `/barq/diag` `Float64MultiArray`, **17 values**:
  `[0..11]` load % (ID order), `[12]` vbus V, `[13]` current A, `[14]` temp_max °C,
  `[15]` fault byte, `[16]` STATE age ms.
- `package.xml`/`CMakeLists.txt`: add `std_msgs`, `sensor_msgs`.
- Emulator verification (before any hardware): `/barq/diag` shows vbus 7.4, temp 25,
  loads 0; `/imu/data` identity quaternion, az ≈ 9.81.

#### 5b. Software E-stop = the stop LADDER (documented as it actually is)
**Correction to a tempting assumption**: killing the *gait node* does NOT stop CMD
frames. `ik_node` keeps streaming its last targets at 50 Hz, the controller keeps
commanding, `BarqSystem::write()` keeps emitting CMD at 100 Hz — the firmware deadman
never trips. Killing the gait node is a **HOLD** (motion stops, stance held, torque
ON). The chain that reaches the 200 ms firmware deadman is killing the
**ros2_control_node** — i.e. Ctrl-C on the `real.launch.py` terminal. The ladder,
weakest to strongest (memorize; drill in §6):

| Rung | Action | Result | Latency |
|---|---|---|---|
| 1 | stop teleop / zero `/cmd_vel` | walks → stands (gait deadman) | ≤ 1.0 s |
| 2 | kill gait (+ik) nodes | **HOLD**: freeze in stance, torque ON | < 0.1 s |
| 3 | **Ctrl-C the launch terminal** (kills ros2_control_node → CMD stream stops) | firmware deadman → ramped torque-off → **LIMP** (robot settles/collapses — pad + spotter) | ~200–300 ms |
| 4 | master switch | power gone | instant |

Bonus auto-rungs already in the stack: serial/firmware death → `BarqSystem::read()`
staleness error at 300 ms → controllers stop → (CMD stops) → firmware deadman; and
the firmware 200 ms deadman itself is never disabled (00 §3). No new E-stop code is
required for P4 — a dedicated `/estop` service on the hardware interface is invasive
and adds a failure mode; the ladder above is the v1 stop system. Keep the launch
terminal visible and reachable at all times; rule: **operator's hand returns to that
terminal whenever the robot is on the floor.**

#### 5c. P4-CODE-5 — fall detection (param-gated, ~15 lines + wiring)
`barq_control/barq_control/state_estimator_node.py`:
- params: `fall_detect` (bool, default **False**), `fall_thresh_rad` (0.8),
  `fall_hold_ms` (100).
- in `_on_imu`: compute `roll = atan2(2(wx+yz), 1−2(x²+y²))`,
  `pitch = asin(clamp(2(wy−zx), −1, 1))`; if `|roll| > thresh or |pitch| > thresh`
  continuously for > `fall_hold_ms` (track first-exceed time) → publish
  `std_msgs/Bool True` on `/fall_detected` (then keep publishing True each tick —
  latched) + `get_logger().error` once. (~12 lines)
- `gait_planner_node.py`: param `honor_fall` (default True); subscribe
  `/fall_detected`; on True set `self.halted = True`; first line of `_tick()`:
  `if self.halted: return` → `/foot_targets` stops → ik holds last stance = HOLD,
  and the operator executes ladder rung 3/4. (~6 lines)
- Margins: threshold 0.8 rad = 46°; designed stance pitch is 0.079 rad — 10× margin.
  Trot heave/turn dynamics come nowhere near it; if false positives appear anyway,
  raise `fall_hold_ms` 100 → 200 FIRST, then thresh 0.8 → 0.9 (one knob per run).
- Why HOLD, not limp, on a fall: torque-off mid-fall flops harder onto the deck and
  gears; freeze + human kill is the v1 judgment — revisit after the first real fall
  (TBD row 4).
- Unit test: synthetic Imu quat at 50° roll for 150 ms ⇒ one True; 50 ms blip ⇒ none.

#### 5d. Thermal / current watchdog (Jetson side)
v1 = **operator watchdog**: a terminal with `ros2 topic echo /barq/diag` is part of
every floor session; thresholds card:

| Quantity | Warn | Stop the session | Source |
|---|---|---|---|
| temp_max (°C) | **60** | **65** (let cool to < 45) | `/barq/diag`[14] |
| vbus (V) | 13.8 (= firmware fault bit2) | **13.6 floor** → G4.8 sequence | `/barq/diag`[12] |
| current (A) | sustained > **TBD row 1** | 2× TBD for > 10 s | `/barq/diag`[13] |
| fault byte | any bit ≠ 0 | bit0/bit1/bit2 = stop now | `/barq/diag`[15] |

v1.1 (optional, P4-CODE-6, ~40 lines, recommended before the 30 min G4.5 run):
`barq_control/barq_control/diag_watchdog_node.py` — subscribe `/barq/diag`; WARN log
at the warn column; at any stop condition publish `std_msgs/Bool True` on
`/halt_gait` (latched) + terminal bell. `gait_planner` subscribes `/halt_gait` with
the SAME handler as `/fall_detected` (3 extra lines). Params mirror the table;
`current_trip_a` ships as 0.0 = disabled until TBD row 1 is measured.

### 6. Drills (the gates)
Drills are rehearsals, not tests of the robot — repeat until boring.

**G4.6 — stop-ladder drill** (robot standing on pad, spotter):
1. Rung-3 drill ×3: phone at 240 fps framing BOTH the operator's hand on the launch
   terminal and the robot. Ctrl-C. Measure keypress → first visible leg yield:
   **< 300 ms** each time (≤ 72 frames @ 240 fps). The robot settles onto the pad —
   spotter guides, doesn't catch.
2. Rung-2 verification ×1: kill ONLY the gait+ik nodes → robot must **HOLD** stance
   (torque on, no limp ≥ 10 s). This drills the HOLD/LIMP distinction into the crew.
3. Restart the stack from scratch after each kill (that's part of the drill: cold
   restart < 2 min).

**G4.7 — fall-detect hand-tilt** (on the CRADLE, `fall_detect:=true`,
`honor_fall:=true`; publish a slow `/cmd_vel` so feet are cycling):
Tilt the body by hand past ~50°: nose-down, nose-up, roll-left. Each time:
`/fall_detected` flips True within ~200 ms of crossing the angle, `/foot_targets`
stops (feet freeze), log line appears. ×3 orientations, then reset (restart nodes),
×3 again. 6/6 trips, zero trips during a normal 60 s air-walk before/after.

**G4.8 — battery-floor drill** (run it the day the pack rests at ~13.7 V):
From walking, execute the full floor-policy sequence, timed, target ≤ 2 min:
1. Announce "battery floor". Zero cmd → robot stands.
2. Kill gait planner; restart ik_node lowered:
   `ros2 run barq_control ik_node --ros-args -p stance_height:=0.10` → body settles
   to ~0.112 m (sit-down).
3. Spotter hand under belly → Ctrl-C launch (rung 3) → limp onto pad.
4. Master switch OFF. 5. Jetson: `sudo shutdown -h now`. 6. Pack to LiPo bag;
   storage-charge if idle > 3 days (00 §3). Log resting voltage before/after.

## Acceptance gates
- **G4.6**: rung-3 limp < 300 ms, ×3 measured on video + rung-2 HOLD verified.
- **G4.7**: 6/6 hand-tilt trips on the cradle, 0 false trips in 2 × 60 s air-walks.
- **G4.8**: complete sit-down-and-shutdown ≤ 2 min at ≤ 13.7 V resting, no step
  skipped (checklist printed and ticked during the drill).
- Estimator (records, not blockers for P4 exit): §3 < 10 % position / < 10° yaw on
  hard floor ×3 — if missed, see Fallback; P4 may exit with the number documented
  ≥ 10 % only if the failure is understood (slip) and parked as the v2 question.

## Fallback ladder
- **Drift > 10 % (hard floor)**: A — §2 axes/sign audit (mount mapping wrong explains
  everything; fix firmware quat mapping). B — zero-offset audit (FK garbage in,
  odometry garbage out; P2 re-calib worst joints). C — 10–15 % with visible slip:
  accept + document, park v2 (slip rejection / P6-05 rung 2 yaw feedback first).
  Switch: 2 attempts per rung.
- **Fall-detect false positives**: hold_ms 100 → 200, then thresh 0.8 → 0.9, one
  knob per run; if still false-positive, check `/imu/data` for quat glitches (§2
  report-type TBD) before touching thresholds again.
- **G4.6 limp > 300 ms**: confirm firmware deadman constant (200 ms, `loop_core.h`),
  check CMD actually stops (serial LED / emulator test), re-measure with a cleaner
  video sync. A real > 300 ms with CMD stopped = firmware ramp too slow → P3
  regression, fix there.

## Rollback
Everything in this file is additive and param-gated: `fall_detect:=false`,
`honor_fall:=false`, watchdog node not launched, P4-CODE-1 publishers are passive
(read-only telemetry). Reverting = stop launching the extras; no robot behaviour
changes. Git-revert the code additions only if they break the build, never "to be
safe" — telemetry must stay.

## TBD table
| # | Unknown | Procedure |
|---|---|---|
| 1 | Sustained walking current bar (watchdog `current_trip_a`) | P3 soak value if recorded; else mean/p95 of `/barq/diag`[13] over 02's G4.5 endurance bag → set warn = p95 × 1.3 |
| 2 | BNO085 SH-2 report in firmware (mag-fused vs game rotation vector) + static yaw wander | §2 10-min wander test; if > 2°, switch report in the P3 IMU stub, re-test |
| 3 | Carpet drift % vs hard floor | §3 ×3 each surface |
| 4 | Fall response: HOLD vs limp | revisit after first real fall — compare damage/behaviour, record decision ADR-style |

## Artifacts → docs/05_RESEARCH_LOG.md
§2 yaw table + wander curve; §3 six L-course rows (surface, error %, yaw °); drill
videos (keep the G4.6 frame counts in the log); the ticked G4.8 checklist; thresholds
card final values; one research-log entry per session; commit + push.

## Escape hatch
The safety drills (G4.6–G4.8) have no bypass — if a drill cannot pass, the robot does
not walk on the floor, full stop; fix the failing layer (firmware deadman → P3;
telemetry → §5a; procedure → re-drill). The ESTIMATOR being out of spec does NOT
block P4 exit: document the measured drift, park the v2 question in
`docs/04_OPEN_QUESTIONS.md`, and proceed — P5 SLAM corrects odometry with scan
matching, and P6-05 rung 2 is the designed consumer of a better estimate. If blocked
≥ 2 sessions on telemetry itself, fall back to bench-only verification (emulator +
`st3215_diag` between runs) and keep sessions short (< 10 min walk time) until
`/barq/diag` is trustworthy.
