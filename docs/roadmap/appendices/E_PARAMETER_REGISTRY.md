# Appendix E — Parameter Registry

> Appendix · verified against repo @ 4ea53a0

Every tunable that exists in the repo TODAY, with its actual current value, plus a FUTURE-TBD
block for constants the roadmap will create. **Scope**: `sim` / `hw` / `both` (both = consumed in
simulation and on the real robot; the PTY emulator counts as hw-path). **Last** = the D-number
that last changed it, `initial` (unchanged since introduction), or `meas. <date>` (measured value,
see `docs/05_RESEARCH_LOG.md`). TBD = must be MEASURED, never guessed; the row names the producing
procedure. File paths are relative to `~/barq_ws/src/`.

## 1. KINEMATICS / MODEL — `barq_description/config/robot_params.yaml`

| Parameter | Value | Scope | Consumed by | Re-derive via | Last |
|---|---|---|---|---|---|
| body length/width/height | 0.258 / 0.117 / 0.085 m | both | URDF collision box; docs | CAD/URDF change only | D-005 |
| body mass | 1.42 kg (incl. electronics, fasteners, 512 g battery) | both | URDF inertial; P6 model parity | re-weigh on hardware change | meas. 2026-06-11 |
| link masses coxa/femur/tibia | 0.0733 / 0.1536 / 0.030 kg (×4; total ≈ 2.448) | both | URDF inertials; P6-02 mass parity check | re-weigh | meas. 2026-06-11 |
| actuators peak_torque_nm | 2.94 (30 kg·cm @12 V) | both | URDF `effort`; sim envelope; P4 load budgets | bench stall test if ever doubted (P2-01 T-rows) | initial (conf. D-018) |
| actuators max_speed_rad_s | 4.71 (0.222 s/60° @12 V no-load) | both | URDF `velocity`; sim slew cap | P3-03 bench sweep | D-018 |
| legs exact constants: knee_x_front 0.01744 (rear −0.01744) · knee_y 0.0430692 · ankle_x 0.018944 · ankle_y 0.0324 · femur_z 0.1 · lateral_offset 0.0754692 · tibia_length 0.100 | (listed) m | both | `leg_kinematics.fk_exact/ik_exact` → ik_node, gait, estimator; locked by `test_exact_kinematics.py` | only via URDF change + test re-lock | D-014 |
| legs legacy coxa_length/femur_length | 0.0465 / 0.107 m | — | tests only — **not-for-control** (up to 3.4 cm foot error) | n/a | D-014 |
| hip_offsets FL/FR/RL/RR | [0.108484, 0.0171913, 0.00022012] / [0.108484, −0.0148092, 0.00022176] / [−0.108371, 0.0167905, 0.00022176] / [−0.108371, −0.01521, 0.00022012] (L/R asymmetry as-designed) | both | gait, ik, estimator (loaded at node start) | URDF joint origins are the source | D-005 |
| servos id 0–11 | FL/FR/RL/RR × coxa/femur/tibia order (one driver board per leg) | hw | Teensy bus driver (P3-01 `firmware_config.h`); `st3215_diag.py plan` | P2-01 ID assignment (one servo connected at a time) | initial |
| servos direction | +1 all except FR_coxa, RR_coxa = −1 | hw | Teensy CMD→counts mapping (mirroring absorbed at hw layer) | P2-03 §direction validation | D-001 |
| servos **zero_offset** | **all 0.0 = TBD placeholders** | hw | Teensy counts↔rad conversion | **→ P2-03 §1 fills all 12 rows** (assembled-neutral residuals; magnitude ≤ half horn-tooth pitch + jig tolerance) | TBD → P2-03 |
| servos min/max_angle | coxa ±0.785 · femur ±1.571 · tibia [−2.2, 0.0] rad | hw | Teensy clamp; servo-side EPROM limits (written in P2-03 §5) | D-012 judgment; re-judged only by the G2.6 fold test | D-012 |

## 2. CONTROL

| Parameter | Value | Lives in | Scope | Consumed by | Re-tune via | Last |
|---|---|---|---|---|---|---|
| controller_manager update_rate | 100 Hz | `barq_bringup/config/ros2_controllers.yaml` | both | gz plugin loop AND ros2_control_node (real); CMD stream rate on hw | fixed by protocol design (Teensy superloop is 500 Hz, see §5) | initial |
| **position_proportional_gain** | **0.6** (k=60/s; default 0.1 = soft fallback, A10) | same yaml (`gz_ros2_control:`) | sim-only | vendored `external/gz_ros2_control` plugin | **→ P3-03 bench ID supersedes** (sim step/sweep vs `st3215_diag sweep`) | D-018 |
| gait period | 0.5 s (also `gait_period` launch default) | `barq_control/gait_planner_node.py` declared param | both | trot phase generator | D-019 method: sweep + `sim_walk_metric` / P4-02 on hw | D-019 |
| gait duty | 0.6 (launch `gait_duty`; 0.55 = dead-straight open-loop, Q-016) | gait_planner param | both | stance fraction; speed/veer trade | D-019 map; re-measure on hw (P4-02) | D-019 |
| gait step_height | 0.02 m | gait_planner param | both | swing apex (front clearance reach-capped ~20 mm — A9) | constraint: stand − step ≥ ~0.095 m (tibia −2.2 at apex) | D-014 |
| gait stand_height | 0.13 m | gait_planner param | both | stance depth | same constraint; deeper crouch = more stance torque (A16) | D-014 |
| gait rear_raise | 0.02 m (≈ +4.5° nose-down; must match ik_node row) | gait_planner param | both | stance trim — load forward | A18 trim ladder (+0.005 steps, cradle first) | D-016 |
| gait rate | 50.0 Hz | gait_planner param | both | /foot_targets stream | n/a | initial |
| gait forward_sign | +1.0 (+x cmd = body +X) | gait_planner param | both | cmd_vel→body mapping | only if frame convention re-decided (Q-012) | D-015 |
| gait cmd_timeout | 1.0 s (gait-level deadman; layer above the 200 ms firmware deadman) | gait_planner param | both | zero cmd on silent /cmd_vel | n/a — do not lengthen | initial |
| ik stance_height / rear_raise | 0.13 / 0.02 m (mirror gait rows) | `barq_control/ik_node.py` params | both | default stance before /foot_targets | change TOGETHER with gait rows | D-014 / D-016 |
| ik knee_bend | −1.0 (legs fold forward) | ik_node param | both | IK elbow branch | physical config — fixed | D-009 |
| ik clamps | coxa ±0.785 · knee ±1.57 · **ANKLE_MIN/MAX −2.2/0.0** | ik_node module constants | both | last-line software clamp before the controller | D-012; re-judge only via G2.6 | D-012 |
| ik tick rate | 50 Hz (hardcoded `create_timer(0.02)`) | ik_node | both | command stream rate | n/a | initial |
| estimator rate | 50.0 Hz | `barq_control/state_estimator_node.py` param | both | integration step | n/a | D-017 |
| estimator publish_tf | False default (sim.launch sets True when `odom_source:=estimated`; hw: estimator owns odom→base_link) | estimator param | both | TF ownership (exactly ONE owner — A5) | P5-02 wiring | D-017 |
| estimator velocity low-pass | a = 0.4 (**hardcoded constant in `_tick`, not a ROS param**) | estimator code | both | gait-ripple rejection | P4-03 drift measurement before promoting to param | D-017 |

## 3. URDF / SIM — `barq_description/urdf/barq.urdf.xacro`, `barq_bringup/launch/sim.launch.py`

| Parameter | Value | Scope | Consumed by | Re-derive via | Last |
|---|---|---|---|---|---|
| joint limits: hip / knee | ±0.785 / ±1.57 rad (all 12 `<limit>` + ros2_control min/max) | both | engine + controller clamps | D-012 judgment | D-012 |
| joint limits: ankle | URDF lower −2.2, **upper +1.57** — KNOWN DELTA: working range is [−2.2, 0] (D-012); ik clamps at 0.0 and P2-03 writes servo limits to the D-012 range; the URDF + side is unused headroom | both | engine + command interface | G2.6 re-judgment propagates here too | D-012 |
| joint effort / velocity | 2.94 N·m / 4.71 rad/s on all 12 | both | engine torque/slew caps (verified engine-side, D-018) | P3-03 bench | D-018 |
| joint dynamics | damping 0.05, friction 0.0 | sim | engine | P6-02 adds armature 0.001 (TBD from bench ID) | initial |
| foot contact sphere | r = 0.012 m at tibia tip (0, 0, −0.1) | sim | contact physics; P6-02 model parity | n/a | initial |
| foot_mu (xacro + launch arg) | default 0.9 (≈ TPU on wood/tile; μ-sweep knob, D-018; realized speed is μ-INVARIANT — A9) | sim | tibia `<mu1>/<mu2>` | measure real pad on hw if ever needed; DR'd 0.3–1.2 in P6-02 | D-018 |
| spawn z | 0.17 m (≈ 3 cm drop onto stance legs; A24) | sim | `ros_gz_sim create` | n/a | D-014 |
| stance initial_values | front (0, 1.047531, −1.928768) · rear (0, 0.911998, −1.652637) — rear = rear_raise trim | both | ros2_control state init; P2-03 pose targets; P4-01 hand-fold target | re-derive from ik if stance params change | D-014 / D-016 |
| laser mount | xyz (−0.04, 0, 0.066), rpy (0, **−0.0785**, 0) = −4.5° wedge countering the D-016 nose-down trim; mass 0.047 kg | both | TF base_link→laser; SLAM scan plane; P5-01 §mount transfers this to the physical bracket | re-level check at P5-01 mount step | initial (Q-015 spec) |
| sim lidar sensor | 2160 samples @ 10 Hz, range 0.15–25.0 m, σ 0.02 (STL-27L twin) | sim | /scan via bridge | P5-01 §1.1 delta table if LD19/D500 bought | initial (Q-015 spec) |
| sim IMU sensor | 100 Hz, gyro σ 0.002, accel σ 0.02 (BNO085-class) on base_link, topic /imu | sim | /imu/data via bridge | P1-04 real noise figures if needed | initial |
| xacro arg mode | mock \| gazebo \| real, default `mock` — only the `<hardware>` plugin changes | both | launch files | n/a | D-006 |
| xacro arg device | default `/dev/ttyACM0` (mode:=real serial port or emulator PTY; post-P1 udev: `/dev/teensy`, A19) | hw | `barq_hw/BarqSystem` | P1-03 udev rules | D-020 |
| BarqSystem state_timeout_ms | 300 ms (no STATE ⇒ refuse activation / mid-run ERROR stop — A23) | hw | barq_hw plugin param (in URDF) | do not lengthen — fix the link | D-020 |
| sim.launch args (defaults) | world_file `barq_world.sdf` · gui `false` · gait `false` · slam `false` · nav `false` · odom_source `ground_truth` · foot_mu `0.9` · gait_duty `0.6` · gait_period `0.5` | sim | full arg matrix + crib §SIM | n/a | D-019 era |
| real.launch args (defaults) | device `/dev/ttyACM0` · gait `false` — line IDENTICAL for emulator and Teensy (drop-in contract) | hw | crib §REAL | n/a | D-020 |

## 4. PERCEPTION / NAV — `barq_bringup/config/barq_slam.yaml`, `barq_nav2.yaml` (sim-tuned; P5-02 §1.1 derives `*_real.yaml` copies — do NOT edit these in place for hardware)

| Parameter | Value | Scope | Consumed by | Re-tune via | Last |
|---|---|---|---|---|---|
| slam resolution | 0.05 m | sim→hw | map grid | transfers as-is | initial |
| slam min/max_laser_range | 0.3 / 20.0 m (min must stay above body radius — A25) | sim→hw | scan window | P5-01 §1.1 if fallback lidar | initial |
| slam map_update_interval / transform_publish_period | 2.0 s / 0.05 s | sim→hw | update cadence; map→odom TF | P5-02 | initial |
| slam minimum_travel_distance / heading | 0.2 m / 0.3 rad (integrate across gait cycles, not per wobbling scan) | sim→hw | update trigger (A25 #2) | P5-02 | initial |
| slam frames / scan_topic / mode / use_sim_time | odom · map · base_link · /scan · mapping · true | sim | TF contract (bare names — A5) | `_real.yaml`: use_sim_time false; scan_topic per P5-01 §5 | initial |
| nav2 FollowPath.desired_linear_vel | 0.22 m/s — **hardware starts 0.10** (P5-02 §4 ladder 0.10→0.15→0.22) | sim | RPP controller | `phase-5-perception-autonomy/02` §4, re-gating G5.5 each step | initial |
| nav2 velocity_smoother.max_velocity | [0.22, 0.06, 0.5] — hw cap must match the 0.10 ceiling ([0.10, 0.06, 0.5]) | sim | the ONLY other place speed lives (crib §SLAM/NAV) | change BOTH keys together | initial |
| nav2 odom_topic (bt_navigator + velocity_smoother) | **/odom_gt — sim-only topic; hardware = /odom_est** (P5-02 §1.1 delta; /odom_gt does not exist on hw) | sim | BT + smoother feedback | P5-02 `_real.yaml` | initial |
| nav2 cost scaling (RPP) | cost_scaling_dist 0.45 · cost_scaling_gain 0.8 (+ regulated min radius 0.4, min speed 0.06) | sim→hw | obstacle-proximity slowdown | transfers; retune only on G5.5 evidence | initial |
| nav2 lookahead | 0.35 m (min 0.2 / max 0.5, lookahead_time 2.0) | sim→hw | RPP carrot | P5-02 §4 | initial |
| nav2 progress / goal | progress 0.05 m per 30 s (slow-walker patience) · xy_goal_tol 0.15 · yaw_goal_tol 0.5 | sim→hw | earned their place in the sim course | transfers as-is | initial |
| costmaps | robot_radius 0.18 · resolution 0.05 · inflation_radius 0.30 · cost_scaling_factor 3.0 · raytrace 12.0 / obstacle 10.0 | sim→hw | local (3×3 rolling) + global | transfers | initial |
| smoother accel caps | max_accel [0.3, 0.3, 1.5] · max_decel [−0.5, −0.5, −2.5] | sim→hw | cmd_vel shaping into the gait | P4-02 hw gait response | initial |

## 5. FIRMWARE / PROTOCOL — `barq_firmware/src/`, `docs/06_PROTOCOL.md`

| Parameter | Value | Lives in | Scope | Consumed by | Last |
|---|---|---|---|---|---|
| superloop period | LOOP_PERIOD_US = 2000 (500 Hz) | `main.cpp` | both (emulator shares LoopCore) | servo bus cadence budget (P3-01 wire-time math) | initial |
| STATE telemetry period | kStatePeriodUs = 10000 (100 Hz) | `loop_core.h` | both | BarqSystem freshest-STATE reads; ~10.3 kB/s | initial |
| **firmware deadman** | kDeadmanUs = 200000 (**200 ms**, fault bit3, torque off) — NEVER disabled, even "temporarily" (00 §3) | `loop_core.h` | both | safety layer below the 1 s gait deadman and 300 ms stale-link stop | initial |
| frame format | magic 0xBA51 · ver 0x01 · CRC16-CCITT-FALSE (0x1021/0xFFFF) · len ≤ 200 · resync decoder | `protocol.{h,cpp}` ↔ `barq_protocol.py`, golden-vector-pinned | both | change ONLY both impls + vectors together | initial (v1) |
| CMD scaling | targets[12] int16 **mrad** (int16-mrad floor = the 3 mrad round-trip seen in 9/9) | 06_PROTOCOL.md | both | BarqSystem ↔ LoopCore | initial (v1) |
| STATE scalings | pos mrad · vel **10 mrad/s** · load **0.1 %** · quat **×1e-4** · gyro mrad/s · accel **cm/s²** · vbus **mV** · current **mA** · temp_max °C (int8) · fault bits 0–3 | 06_PROTOCOL.md | both | estimator/diagnostics decode; P4-03 `/barq/diag` | initial (v1) |
| power-monitor naming | protocol says "(INA226)" — STALE: INA260s are owned; fields are chip-agnostic (README hard-truths; P1-02 designs within 15 A/unit) | 06_PROTOCOL.md comment | hw | doc fix only, no field change | flagged here |

## 6. FUTURE — TBD (no repo value yet; the named doc PRODUCES the value, then a row moves up)

| Future parameter | Planned value/ladder (from the phase doc — verify there) | Producing doc |
|---|---|---|
| Buck output set-point + ripple + continuous rating | 12.0 V (trim-pot), output ≥ 11.7 V under worst-case load; rating ≥ 1.5× measured rail draw (TBD-3, expect 20–30 A class); ripple TBD-8 | `phase-1-power-electronics/01_POWER_TREE.md` |
| Brownout/battery-voltage ladder | warn **14.0** → fault bit2 **13.8** (INA260 ALERT latch) → operational floor **13.6** (manual stop, G4.8 drill) → firmware hard floor **13.2** (kVbusFloorMv 13200, 5 consecutive samples) | P1-01 G1.3 · `…/02_MONITORING_INA260.md` · `phase-3-firmware-integration/02` · P4-01/P4-03 |
| Servo torque-limit staging | 30 % → 60 % → 100 % via `EPROM_MAX_TORQUE` (addr 0x10, units 0.1 %, persists) — first-stand ramp | `phase-4-standing-walking/01_FIRST_STAND.md` §2 (P4-CODE-2) |
| Fall detection | `fall_detect` False (armed at G4.7) · `fall_thresh_rad` **0.8** (46° = 10× stance pitch 0.079) · `fall_hold_ms` 100 · gait `honor_fall` True | `phase-4-standing-walking/03_ESTIMATOR_AND_SAFETY.md` §5c (P4-CODE-5) |
| Servo thermal thresholds | **60 °C** warn / **65 °C** stop, cool to < 45 °C — become firmware faults (A16) | P4-03 telemetry table; firmware wiring per P3 |
| zero_offset ×12 | replace the 0.0 placeholders in §1 | `phase-2-calibration-assembly/03` §1 (G2.x) |
| Bench-ID'd sim gain | supersedes position_proportional_gain 0.6 with the bench-matched value | `phase-3-firmware-integration/03_HIL_VALIDATION.md` |
| Bus baud decision | stay 1 Mbaud (10 µs/byte) or drop to 500 k + read-strategy change; EPROM_BAUD value table TBD-2; max harness length TBD-13 | `phase-3-firmware-integration/01` · `phase-1-power-electronics/03` |
| IMU interface + mount transform | I2C → SPI → UART-RVC ladder outcome; chosen rotation-vector variant; axes→base_link rotation (A17) | `phase-1-power-electronics/04_BNO085_BRINGUP.md` |
| Hardware nav speed | 0.10 → 0.15 → 0.22 m/s ladder, re-gated each step | `phase-5-perception-autonomy/02` §4 |
| RL action scale | **0.25**: `q_target = q_stance + 0.25·a`, clamped to ctrlrange | `phase-6-rl/02_ENV_AND_REWARD_SPEC.md` §action |
| RL domain-randomization ranges | μ 0.3–1.2 (narrow 0.5–1.0) · base mass ±15 % (→ ±10 %) · action delay 0–30 ms · full table in the doc; rule: **center DR on measured values as P3/P4 produce them** | `phase-6-rl/02` §8 |

## The rule (read this twice)

**Any value changed anywhere must update BOTH the owning file AND its row here, plus one line in
`docs/05_RESEARCH_LOG.md` (what was believed / measured / changed).** A registry row that disagrees
with the repo is not a paperwork problem — it is a live failure-tree symptom (risk R-20): the next
procedure executed from this table will act on a constant the robot no longer has. Treat a found
mismatch exactly like an appendix A entry: reproduce (re-read the owning file), fix BOTH ends in
the same commit, log it. When in doubt, `00_DOOMSDAY_PROTOCOL.md` §2 applies — verify at the lowest
layer that can lie, which for parameters is the owning file at the pinned commit, never this table.
