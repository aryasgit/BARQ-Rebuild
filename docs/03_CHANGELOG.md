# BARQ — Changelog

Dated log of concrete repo changes. Newest first.

---
## 2026-06-13 — Doomsday Roadmap shipped: docs/roadmap/, 33 docs, P0->P7 + appendices (D-021, D-022)
Full post-sim execution bible, written for zero-LLM execution: environment rebuild from bare
metal; power tree (4S + mandatory 12 V buck — the power architecture decision is now recorded as
D-021, incl. INA260-not-INA226 corrigendum); Teensy/bus/IMU bringup; per-servo bench calibration
+ assembly-at-midpoint discipline + D-012 fold check; firmware stub-fill specs (ST3215 sync-write
timing budget, SH-2 IMU, INA260) + HIL graduation incl. re-tuning the sim servo gain to the
bench; first-stand/first-steps protocols with tuning decision trees; lidar purchase gate
(24k INR ceiling) + on-robot SLAM/nav2 + Orin compute-budget ladder; RL with THREE compute
tracks (cloud / local RTX / Mac-CPU), code-ready env+reward spec, training recipes,
sim2real+deployment (rl_policy_node + shared rl_obs.py spec), and a 6-rung no-RL bypass ladder;
field acceptance courses replicating the sim obstacle course + operations runbook. Conventions
throughout: acceptance gates G<phase>.<n>, A->B->C fallback ladders, TBD-tables naming the
producing procedure. Review pass fixed one stale code comment (gait_planner reach-floor 0.095 ->
0.108, D-019 value). Agents' cross-checks caught: kill-gait-node does NOT trip the deadman
(stop-ladder documented in P4-03 and adopted by P6-04), estimator needs the /imu/data bridge on
hardware (P4-CODE-1), nav2 yamls carry sim-only odom_topic/use_sim_time (P5-02 deltas).

---
## 2026-06-11 — Stage 4: hardware interface DONE, integration-tested against real firmware logic (D-020)
- **barq_hw** package: `BarqSystem` SystemInterface (protocol v1 over serial; activation
  requires live STATE -> controllers start from measured pose; stale-link ERROR; name-keyed
  joint->servo-slot map) + `teensy_emulator` (the REAL firmware LoopCore on a PTY).
- **Firmware refactor**: superloop logic -> `loop_core.{h,cpp}` shared verbatim by `main.cpp`
  (Arduino shim) and the emulator. pio native 6/6, teensy41 compiles.
- **mode:=real** wired in the xacro (`device` arg) + `real.launch.py` — the LITERAL robot
  launch line, used by the integration test today and the robot later.
- **integration_pty.py 9/9**: Python codec vs firmware bench (PING/PONG, CMD->STATE echo
  3 mrad, deadman bit3 latch) + full controller_manager stack on the PTY (100 Hz
  /joint_states, round-trip 3 mrad). Full-gait rehearsal on the emulator: knee swing
  ~330 mrad, /joint_states 99.96 Hz (sigma 0.43 ms).
- Hardware day remainder: flash LoopCore, fill servo_bus_*/imu_read/power_read stubs,
  `device:=/dev/ttyACM0`. (v1 = joints only; IMU broadcaster is a hardware-day add.)

---
## 2026-06-11 — Sim made ST3215-true: actuation honesty + swing-drag fix (D-018, D-019)
Sim-fidelity sweep so hardware drop-in meets a calibrated world, not a flattering one:
- **Actuator envelope engine-verified** (`ign sdf -p`): effort 2.94 N·m, velocity 5.24 -> **4.71
  rad/s** (Waveshare 0.222 s/60deg @12 V), foot mu — all 12 joints, in the SDF the engine loads.
- **Servo stiffness now real AND configurable**: found ign_ros2_control 0.7.x constructs its node
  before reading `<parameters>` -> `position_proportional_gain` unreachable (stuck k=10/s, ~6x
  soft). **Vendored + 3-line patch** `external/gz_ros2_control` (PROVENANCE.md, BARQ.patch);
  launch shadows the /opt binary via IGN_GAZEBO_SYSTEM_PLUGIN_PATH. Now k=60/s in
  ros2_controllers.yaml: step = 50 ms rise (51 theoretical), trot tracking 17.8 mrad mean RMS
  (was 55.4). Bench-matchable via the same metrics in st3215_diag.
- **Q-013 SOLVED**: friction sweep (foot_mu arg, 0.25/0.5/0.9) showed realized speed mu-invariant
  ~47% -> swing-foot drag, not slip (Coulomb ratio cancels mu). Front clearance is reach-capped at
  20 mm (2 mm from the tibia fold limit). **Swing smoothstep** (travel mid-swing, soft touchdown)
  + duty 0.6: ~60% realized; duty 0.55 walks dead straight at 47% (`gait_duty` launch arg; Q-016
  tracks the yaw-vs-duty mechanism).
- New diagnostics: `sim_actuation_probe.py` (step/track), `sim_walk_metric.py` (WALK regression
  line), `analyze_track_bag.py` (offline tracking from ros2 bag — immune to rclpy starvation,
  which produced misleading numbers twice on the loaded Jetson).
- Launch args added: `foot_mu`, `gait_duty`, `gait_period`. Gait unit tests: 8/8 green.

---
## 2026-06-11 — Stage 3 opened: Jetson<->Teensy protocol, test-first on BOTH ends
The binary protocol (docs/06_PROTOCOL.md) implemented twice and pinned to shared golden vectors:
- **Python** `barq_control/barq_protocol.py` (CRC16-CCITT-FALSE framing, CMD/STATE/PING-PONG,
  resync-capable stream decoder) — 6 pytest cases incl. corruption-reject + byte-split streams.
- **C++** `barq_firmware/src/protocol.{h,cpp}` (pure C++, no Arduino deps) — 6 Unity tests run
  NATIVELY on the Jetson (`pio test -e native`), asserting the SAME golden bytes as Python.
  (Hand-rolled hex in the first draft had two byte-swaps — generators write vectors, not humans.)
- **Firmware v0 "loopback"** (`barq_firmware/src/main.cpp`): 500 Hz superloop skeleton — decoder,
  200 ms deadman (fault bit3, LED state), 100 Hz STATE telemetry; servo-bus/BNO085/INA226 stubbed.
  Flash-day plan: Stage-4 hardware interface integration-tests against a bare Teensy.
PlatformIO 6.1.19 installed on the Jetson host; gotcha: `test_build_src = yes` required or tests
do not link src/. Protocol scaling: positions mrad(i16), vel 10mrad/s, quat 1e-4, STATE=98 B
payload @ 100 Hz ≈ 10.3 kB/s.

---
## 2026-06-11 — State estimator v1: SLAM now runs on honest legged odometry (D-017)
The last ground-truth seam closed. Added: sim IMU (BNO085-class noise, `/imu/data` — Stage-3
hardware topic parity) + imu-system plugin in both worlds; `state_estimator_node` (stance-diagonal
FK velocity + IMU yaw @ 50 Hz -> `/odom_est`, owns odom->base_link TF when
`odom_source:=estimated`); ground truth always available as `/odom_gt` for A/B; nav2/rviz odometry
displays point at `/odom_gt`. 4 new unit tests (24 pass).

**Bug worth a paper paragraph**: "two lowest feet = stance" always picked BOTH REAR legs — the
D-016 rear_raise trim made rears permanently deepest; per-leg cyclic motion averages zero ->
odometry flatlined (0.0015 m vs 0.86 m truth). Fix: compare trot DIAGONAL z-sums (each diagonal =
one front + one rear, trim cancels). Estimator assumptions must be audited against stance features.

**Measured**: drift 0.075 m over a ~1.6 m lap (~4-5% of distance — typical for kinematic legged
odometry). Honest-SLAM validation: full map built on estimated odom (161x121, 1332 occ); map->odom
correction live at [-0.108, -0.079] ≈ mirror of est-vs-gt error — SLAM absorbing real drift.

Ops gotcha solved: recurring wedged execs = `ros2` CLI swallowing SIGTERM under load + `timeout`
never escalating. ALWAYS `timeout -k 2 N ros2 ...`. (In HANDOFF.)

---
## 2026-06-11 — Dynamic speed (confidence-regulated) + compute-budget findings
Per Aryaman: fast when confident, slow when computing/near obstacles. RPP already supports exactly
this — enabled **cost-regulated velocity scaling** (slows by costmap proximity) on top of the
curvature regulation: desired_linear_vel 0.12 -> **0.22**, cost_scaling_dist 0.45/gain 0.8,
velocity_smoother ceiling 0.22. Applied LIVE to the running course mission via dynamic parameters
(robot visibly sped up mid-tour), persisted in barq_nav2.yaml. Joint-limit envelope verified at the
0.22 ceiling (unit test now covers it; 20 pass).

Compute findings (obstacle-course session): action handshake timed out under full load (sim+SLAM+
nav2+2 GUI renderers) — goal silently lost, robot static. Mitigations now: shed Gazebo GUI during
missions (RViz is the mission view), robust python action client (explicit accept/result) instead of
the CLI. Production note: the two biggest sim-era loads (physics+lidar rendering, GUI rendering)
DO NOT EXIST on the real robot; the Teensy split already isolates hard-real-time from Jetson load.
Course world `barq_course.sdf` (10x8: doorway + slalom + box field) + `world_file:=` launch arg.

---
## 2026-06-11 — AUTONOMOUS NAVIGATION: nav2 mission SUCCEEDED in sim
nav2 added on top of the SLAM stack: `barq_nav2.yaml` tuned for a slow walker (RPP controller,
desired 0.12 m/s, progress checker 5 cm/30 s, goal tol 0.15 m, footprint r=0.18, inflation 0.30,
costmaps from /scan + SLAM map), `nav:=true` in sim.launch.py (nav2_bringup navigation_launch).
Dockerfile: navigation2 + nav2-bringup — its dep tree hits the dustynv CUDA-opencv header
conflict; fixed with --force-overwrite (headers only) + --no-install-recommends. (Build #4
FAILED on this and the failure initially read as success — check the actual log tail, not the
task exit code.) RViz: green /plan path + costmap display; "2D Goal Pose" tool = click-to-navigate.

**First autonomous mission**: NavigateToPose (1.6, 2.2) from origin -> SUCCEEDED, final pose
(1.548, 2.067), error 0.14 m (< 0.15 tol), ~2.6 m walked around a pillar inflation zone.
Full loop: lidar -> SLAM -> costmaps -> NavFn plan -> RPP -> cmd_vel -> trot gait -> IK -> physics.
Also: cmd circle test (0.12 m/s + 0.3 rad/s) closed a clean R=0.4 m circle — v/omega composition
verified in physics.

---
## 2026-06-11 — Three defects found by live scrutiny (deadman, RViz durability, /dev/shm)
Aryaman watched the live demo and caught BARQ wall-grinding forever + an empty RViz. Root causes:
1. **No cmd_vel deadman**: gait_planner held the last velocity forever after a teleop Ctrl-C.
   Added `cmd_timeout` (1.0 s default) — verified: "cmd_vel silent 1.0s - deadman stop".
   Real safety fix; carries to hardware.
2. **RViz late-joiner**: barq_slam.rviz robot_description subscription was Volatile -> late-started
   viewers never got the model. Now Transient Local. Fixed frame odom (map frame only exists
   after SLAM publishes).
3. **Cross-container DDS data loss**: FastDDS uses /dev/shm for same-host transport; containers
   have private /dev/shm -> viewers SAW topics (UDP discovery) but received NO data (tf/clock
   dropped; the "empty RViz"). Fix: run ALL ROS containers with `-v /dev/shm:/dev/shm`.
   Recorded in HANDOFF + memory — applies to every future multi-container setup.
Also: slam yaml min_laser_range 0.3 (clears a startup warning).
Verified end-to-end on screen: map (160x120) + live scan fan + robot model + odometry in RViz,
all statuses Ok, while walking in Gazebo.

---
## 2026-06-11 — Sim perception: lidar + SLAM end-to-end in Gazebo (Q-015 sim-first)
Full 2D-perception pipeline, hardware-free (lidar purchase deferred; STL-27L-class specs):
- URDF: `laser` link (47 g modelled, aft deck, **-4.5 deg counter-wedge**) + `gpu_lidar`
  (360 deg, 2160 samples, 25 m, +noise) + OdometryPublisher (ground-truth odom stopgap), all
  gazebo-mode only; mock/RViz paths untouched.
- World: Sensors render system (ogre2 + `--headless-rendering` EGL — WORKS on Tegra; MESA/EGL
  warnings in the log are non-fatal) + an 8x6 m walled room with pillars (first attempt mapped
  NOTHING: empty world = no lidar features. Mapping needs geometry).
- Launch: /scan + /odom bridges; `slam:=true` -> slam_toolbox (async, barq_slam.yaml);
  `odom_tf` node re-publishing /odom as TF with **forced** frame names — gz plugins model-prefix
  TF frames (e.g. barq/odom), which silently breaks slam_toolbox's odom->laser chain (it dropped
  every scan: "queue is full"). Image: + ros-humble-slam-toolbox, ros-humble-laser-filters.
- Viewer: barq_slam.rviz (Map + LaserScan + RobotModel + Odometry, fixed frame odom).

**Verified (post-Jetson-reboot, clean run):**
- TF odom->laser: z 0.220 m, rotation ~0 — the counter-wedge EXACTLY cancels the D-016 stance
  pitch (designed prediction confirmed in physics).
- /scan 9.7 Hz light / ~7 Hz under full load; slam_toolbox "Registering sensor".
- Drove fwd+arc+fwd; **/map: 161x121 @ 0.05 m = 8.05 x 6.05 m — the room, walls+pillars as
  1127 occupied cells, 16898 free**; map->odom TF live. RTF ~41% with sensors+SLAM+GUIs.
Note: Jetson rebooted mid-session (container Exited 255, /tmp wiped) — VNC/x11vnc needs re-run
after reboots; xorg display config persists.

---
## 2026-06-11 — Strategy: sim-to-the-max; ST3215 bench diagnostics tool; lidar research
Aryaman: no components delivered yet — continue maximizing the sim world (offsets/balance/stability
tunable before hardware); RL outlook improved by sim fidelity. Two artifacts:
- **`diagnostics/` (new)**: `st3215_diag.py` — standalone pyserial implementation of the Feetech
  STS register protocol (1 Mbps) for the Waveshare ST3215 + Serial Bus Servo Driver boards (USB;
  no Teensy/ROS needed). Commands: scan/ping/status/monitor/torque/set-id/calibrate-mid (Feetech
  torque=128 midpoint trick)/move/sweep (tracking-error report)/limits/plan (BARQ ID map 0-11,
  one driver board per leg). README: bench procedure per servo before assembly, calibration data
  flow into robot_params `zero_offset`. Compiles + CLI verified; **hardware paths untested until
  servos arrive**. Honest status: Teensy firmware (Stage 3) / C++ hw interface (Stage 4) still
  unwritten — this is the repo's first hardware-facing artifact; the Teensy will reuse this
  register map.
- **Lidar**: research subagent dispatched on the SLAMTEC RPLidar A2M12 (fit for a 2.45 kg
  quadruped, ROS2 Humble/Jetson wiring, making 2D useful on a walking robot, alternatives,
  Gazebo-first integration). Report lands in chat; decision pending (Q-015).

---
## 2026-06-11 — Stance trim: rear_raise (load-forward, anti backward-tip) — D-016
Aryaman observed the rear legs taking more load in sim and asked for front-more-contracted /
rear-more-relaxed. Implemented as `rear_raise` (default 0.02 m): rear feet 2 cm deeper than front
(rear depth 0.15, front 0.13) -> ~5 deg nose-down pitch, load shifts to the front feet. Raising the
REAR (not dropping the front) keeps the front swing apex inside the tibia envelope (in-plane reach
at q3=-2.2 is 0.1079 m; earlier comment said ~0.095 — corrected). Param in gait_planner + ik_node;
URDF rear initial_values updated to the trimmed stance (knee 0.911998, ankle -1.652637); gait.py
constraint note fixed; new unit test (20 pass; 3 symmetric-stance tests now pin rear_raise=0).

Physics: settle pitch **+4.5 deg nose-down**; walk +0.603 m/10 s (best yet: 0.44 -> 0.52 -> 0.60);
yaw drift -0.018 rad/10 s (was -0.031), lateral ~0.4 mm — the front-loaded stance tracks much better.
Pending (Q-014): Aryaman will supply exact CoM coordinates -> update base_link inertial origin.

---
## 2026-06-11 — Measured masses + real servo torque caps in the model
Aryaman measured (servos+fasteners included): body **1.42 kg**, coxa **73.3 g**, femur **153.6 g**,
tibia **30 g** -> total **2.448 kg** (was ~1.66 with estimates; femur was 3x underestimated).
ST3215 mechanical peak torque 30 kg.cm -> **2.94 N.m** set as `<limit effort>` on all 12 joints
(was a fantasy 10.0; ign_ros2_control really enforces this in physics). Inertia tensors recomputed
from primitive approximations at the measured masses (body box 2.47e-3/8.73e-3/9.50e-3; femur
1.7e-4/1.6e-4/5.3e-5; coxa 3.2e-5/2.0e-5/3.2e-5; tibia 3.3e-5/3.1e-5/3.3e-6 — replacing
placeholders). robot_params: body mass, `link_masses`, new `actuators` section.

Physics A/B (same test as before): settle z **0.141779 identical**, level; walk +0.523 m/10 s
(slightly BETTER than 0.44 — heavier robot has more friction authority); small yaw drift 1.8deg/10 s
(open-loop). Conclusion: servos hold the real robot with ~5-10x stance-torque margin, proven in sim.

---
## 2026-06-11 — Forward direction finalized: +X (forward_sign=+1)
Aryaman watched the Gazebo walk and ruled the -X direction backwards: forward is **+X**, the arc
direction approved in the RViz reversal session. One-parameter fix (`forward_sign` -1 -> +1).
Physics-verified: cmd +x -> **+0.442 m in ~10 s, straight (-5 mm), level**, settle z again 0.141779.
Resolves Q-012 (D-015): FL/FR labels are genuinely the front legs; no rename needed.

---
## 2026-06-11 — Stage 2E COMPLETE: BARQ walks in Gazebo physics, head-first
Multi-day bring-up, three phases:

**1. Gazebo Fortress infrastructure.** Dockerfile: OSRF repo + slim package set
(`ros-gz-sim`, `ros-gz-bridge`, `ign-ros2-control` — NOT the `ros-gz` metapackage: its
demos/image extras pull Ubuntu libopencv-dev which collides with the dustynv image's CUDA
opencv-dev; first build failed exactly there). `barq.urdf.xacro` mode:=gazebo branch
(ign_ros2_control hardware + system plugin reading ros2_controllers.yaml). Collisions
reduced to body box + 4 foot spheres (r=0.012 at the kinematic foot point) — mesh collisions
are slow/unstable for contacts. Offline `barq_world.sdf` (no Fuel). `sim.launch.py`:
gz server (+optional GUI), spawn from /robot_description, /clock bridge, spawners, gait:=true.
GOTCHA: with --network host all containers share one DDS graph — running the RViz demo and
the sim simultaneously cross-polluted /controller_manager etc. One stack at a time.

**2. Adversarial review (25-agent workflow): 3 confirmed findings, 18 refuted.**
(a) startup collapse-then-snap: no initial joint values + spawn 0.25 m; (b) the idealized
leg model was off by up to 3.4 cm at the FRONT feet (knee x-offset +-0.01744 front/rear,
femur 10.7deg in-plane angle, 0.0324 lateral ankle offset all folded into fake "lengths" —
front/rear ASYMMETRIC, support polygon shifted, stand height 11 mm off); (c) zero swing
clearance (step 0.012 = sphere r, apex 0.1 rad from the tibia clamp).
Bonus correction: Humble ign_ros2_control applies position commands as a stiff velocity
loop (JointVelocityCmd = error x update_rate), not literal teleports; initial values DO
use JointPositionReset.

**3. Exact kinematics + fixes (D-014).** `leg_kinematics.py`: fk_exact/ik_exact modelling
the URDF chain exactly (q1-invariant x-offsets, femur in-plane L2P/A2, combined lateral
LAT=0.0754692; URDF +q2 tilts the femur toward -X — the legacy model had this sign
MIRRORED, the root of the direction confusion). Verified against a rotation-matrix
composition of the raw URDF origins to <1e-12 (test_exact_kinematics.py); legacy model kept
but marked not-for-control. ik_node + gait use the exact model; neutral foot = knee-x
forward/back of hip (symmetric support polygon). Gait stand 0.13 / step 0.02 (real
clearance; tibia apex -2.18 within -2.2). URDF: initial_value on all 12 joints = exact
stance (0, 1.047531, -1.928768); spawn z 0.17. gait_planner gains `forward_sign` param.

**Physics results** (headless, ign model --pose):
- Settle: x=-0.000001, z=0.141779 vs predicted 0.142 (0.2 mm!), level, zero transient
  (was: -3.2 cm startup lurch + 11 mm height anomaly — both were the model error).
- Walk cmd_vel +x: **-0.376 m in ~8 s toward -X = HEAD-FIRST, the correct direction**
  (was +X/tail-first; cause was the asymmetric foot placement + scuffing, not the mapping).
  forward_sign=-1 confirmed as default. ~0.047 m/s realized of 0.12 commanded (open-loop
  slip — tuning headroom for later).
19 unit tests pass. Stray ws-root demo files cleaned.

**GUI over VNC (Jetson/Tegra specifics):** Gazebo GUI rendered BLACK with the default ogre2
engine -> `--render-engine-gui ogre` in sim.launch.py fixes it. Robot meshes were invisible in
the GUI (collisions fine) until `IGN_GAZEBO_RESOURCE_PATH` was set to the description share's
parent so `package://barq_description/...` URIs resolve. Camera: `ign service /gui/follow`
(+ /gui/follow/offset) locks the view onto barq. Verified: full meshes render, shadow, RTF ~100%.

---
## 2026-06-10 — Deep crouch: tibia judgment limit -2.2, stand_height 0.115; first push
Team confirmed the ST3215s are **360-deg servos — no hard mechanical stop**; all joint limits are
design judgment (resolves Q-001's premise). Aryaman wanted a much deeper crouch (more femur, more
tibia fold, narrow). Changes:
- Tibia limit -1.57 -> **-2.2 rad (~126deg)** everywhere: URDF ankle joints (x4), ros2_control
  command interfaces (x4), robot_params servo map (x4), ik_node clamp (now asymmetric [-2.2, 0]).
- stand_height 0.16 -> **0.115 m (~28% lower)**; ik_node static stance matches. New floor:
  min 2-link reach at q3=-2.2 is ~0.094 m -> stand-step must stay >= ~0.103 m.
- Regression test bounds updated to [-2.2, 0]; **14 tests pass**; xacro 0 stderr; check_urdf OK.
Live: femur +0.93..+0.98, tibia -1.96..-2.08, diagonals intact. Flag: verify no link collision at
full fold on the physical build (D-012).
Git: `stage-2` pushed to origin (first push) — SSH over 443; all commits authored Aryaman Gupta.

---
## 2026-06-10 — Gait reversed (head-first) + crouched stance
Aryaman (after fold fix): gait stepped toward the tail end; wanted travel toward the head, and a lower
body for stability. Two changes, both at the gait layer (URDF/IK/gait math untouched):
1) `gait_planner_node` now maps cmd_vel (robot-centric, +x = head-first) into body axes by negating
   linear x,y (yaw unchanged) — reverses the traversal arc. Q-012 documents the frame story.
2) Crouch: defaults stand_height 0.18 -> 0.16, step_height 0.03 -> 0.012 (node + gait.py). Constraint
   honored: stand-step >= ~0.147 m, else the swing apex demands tibia beyond -1.57 (leg geometry).
New regression test `test_default_gait_stays_within_tibia_range` (full default cycle, tibia within
[-1.571, 0]) — **14 tests pass**. Live: knees +0.56..+0.75, ankles ~-1.36..-1.55, diagonals intact.

---
## 2026-06-10 — Knee-bend branch flipped: legs now fold FORWARD (Q-010 resolved)
Aryaman (visual check over VNC): all legs folded backward — tibia closed toward the body to the rear;
femur+tibia needed the mirrored pose ("45 -> 135", both joints, all legs). That is exactly the other
analytical IK elbow branch. Fix: default `knee_bend` +1 -> **-1** in `leg_kinematics.ik_leg` and the
`ik_node` param. Stance mirrors (knees -0.73 -> +0.73, ankles +1.52 -> -1.52); foot positions identical
(FK-verified). New unit test `test_forward_fold_branch` (same foot point, q3 <= 0) — **13 tests pass**.
Corroboration: servo config in robot_params has tibia range **[-1.571, 0]** — the -1 branch is the only
one the real hardware can reach (strong evidence toward Q-001's one-directional tibia).
Live-verified walking: knees +0.45..+0.70, ankles -1.03..-1.34 (inside the servo range), trot diagonals
preserved. NOTE: direction-of-travel / which end is +X is deliberately deferred (Aryaman: "that's for
later") — only the fold direction changed; gait paths and URDF untouched.
(An earlier frame-flip attempt (f4cd735) was reverted by request — branch reset to 2982b06 first.)

---
## 2026-06-10 — Stage 2D: trot gait planner (verified — BARQ walks)
Added `barq_control/gait.py`: open-loop trot foot-trajectory generator (diagonal pairs FL+RR / FR+RL,
50% duty, swing arc + stance sweep; step size scales with cmd velocity; neutral stance at zero cmd).
6 unit tests pass (zero-cmd stance, periodicity, diagonal sync, swing lift, step scaling, all targets
IK-reachable). Added `gait_planner_node.py`: /cmd_vel (Twist) -> /foot_targets @ 50 Hz, gait params as
ROS params. `control.launch.py` gains `gait:=true` (implies ik via a PythonExpression OR-condition).
setup.py console_script + geometry_msgs dep.

Lint: brought barq_control fully clean — import ordering (ament style), D213 docstrings, removed unused
imports, line lengths. All 12 tests green (4 IK + 6 gait + flake8 + pep257).

Verified live (headless): /cmd_vel x=0.15 -> all 6 nodes up, /joint_states cycles with correct diagonal
pairing (FL_knee==RR_knee, FR_knee==RL_knee). Tuned defaults stand_height=0.18 / step_height=0.03 so the
ankle stays within +/-1.57 during swing (at 0.16 the legs were too bent to lift without clamping).

Foundation audit (pre-2D): clean build from scratch (5/5), interface counts all 12, barq_hardware (C++)
and barq_sim correctly deferred to Stage 4 / 2E.

---
## 2026-06-10 — Stage 2C: analytical IK node (verified)
Added `barq_control/leg_kinematics.py`: idealized 3-DOF leg (clean link lengths from robot_params;
coxa about X, femur/tibia about Y) with forward kinematics and closed-form analytical IK. Unit tests
`test/test_ik.py` verify FK<->IK round-trip to 1e-9 across the workspace + neutral/unreachable cases
(4 pass). Fixed an angle-wrap (`+acos` branch pushed q1 past pi -> normalize outputs to (-pi, pi]).
Added `ik_node.py`: loads geometry, streams a default stance, subscribes `/foot_targets` (12 foot xyz,
body frame, FL/FR/RL/RR), publishes 12 joint positions to the position controller @ 50 Hz, clamped to
joint limits. `control.launch.py` gains `ik:=true`. Wired barq_control setup.py (console_script) +
package.xml (rclpy/std_msgs/ament_index_python/yaml/barq_description deps).

Verified live (headless): IK node @ 50 Hz; stance hips=0, knees=-0.73, ankles=1.52 (symmetric across
4 legs); commanding feet 0.19 m below hips straightened the legs (knees -0.39, ankles 0.82). Knee-bend
branch = +1 (verify visual direction in RViz, Q-010).

---
## 2026-06-10 — Stage 2B: ros2_control mock loop (verified)
Renamed `barq.urdf` -> `barq.urdf.xacro` (added `xmlns:xacro`, a `mode` arg) and appended a
`<ros2_control>` system block: `mock_components/GenericSystem` (mode=mock), all 12 joints with a
position command interface + position/velocity state interfaces. Added
`barq_bringup/config/ros2_controllers.yaml` (joint_state_broadcaster + joint_group_position_controller
over all 12 joints) and `control.launch.py`. Pointed `visualize.launch.py` at the `.xacro`; `setup.py`
installs `config/`; `package.xml` gains controller_manager / joint_state_broadcaster /
position_controllers / ros2controlcli deps.

**Gotcha fixed:** macro params named `min`/`max` shadow xacro builtins -> "redefining global symbol"
warnings on **stderr** -> the launch `Command` substitution treats any stderr as failure and aborts
before a single node starts. Renamed params to `lower`/`upper` (xacro now emits 0 bytes stderr).

**Verified headless (mode=mock):** 4 nodes up; `joint_state_broadcaster` +
`joint_group_position_controller` both ACTIVE; publishing `{hips=0, knees=+0.3, ankles=-0.6}` to
`/joint_group_position_controller/commands` -> `/joint_states` reflects it. Benign: ros2_control_node
can't set FIFO RT scheduling under Docker (Q-009).

---
## 2026-06-09 — Stage 2A DONE: BARQ rendering in RViz over VNC
After `fix_display.sh`, `:0` came up at 1024x768 with DP-0 forced connected (no more black).
`xrandr --addmode` for 1600x900 was rejected by the NVIDIA driver (no EDID) -> stayed at 1024x768
(usable). Resized `barq.rviz` to fit (1024x728, right dock hidden) and relaunched the `barq_rviz`
container. Confirmed via host screenshot: **RViz shows the full BARQ model** — body, 4 legs,
coxa/femur/tibia, curved feet, TF triads, grid; OpenGl 4.6. Stage 2A visual check PASSED.
(For 1600x900 later: supply a real EDID blob to CustomEDID in the xorg.conf.)

---
## 2026-06-09 — Headless display resolution fix (fix_display.sh)
Root-caused the black screen: Orin is headless (DP-0/DP-1 disconnected, NVIDIA Tegra driver,
`AllowEmptyInitialConfiguration` -> 640x480 default, no EDID on system). `xrandr --fb` enlarges the
buffer but nothing scans it out -> black. Added `~/fix_display.sh` (host): backs up
`/etc/X11/xorg.conf`, writes a config that forces DP-0 `ConnectedMonitor` + a cvt `1600x900_60`
Modeline + `AllowNonEdidModes`, then restarts gdm (autologin=barq recreates the :0 session). Safe:
falls back to 640x480 if the mode is rejected; revert via the backup. Pending: user runs
`sudo ~/fix_display.sh`, then Claude restarts x11vnc + relaunches RViz. No repo files changed.

---
## 2026-06-09 — RViz running over VNC; headless display notes
Brought Stage 2A visualization up live over VNC. Launched the visualize stack in a detached container
(`barq_rviz`) rendering to `:0`: robot_state_publisher loaded all 13 segments; rviz2 got **OpenGl 4.6**
(GPU-accelerated on the Tegra); jsp_gui up. Gotchas (for future reference):
- Headless `:0` defaults to **640x480** (no monitor) -> RViz (1200x800) opened mostly off-screen.
  `xrandr --fb 1600x900` enlarges it without sudo, but GNOME/mutter may need a VNC reconnect to repaint.
- `gnome-screenshot`/`xwd` can't reliably capture direct-GL windows headlessly (return black) — verify
  via the VNC client, not host screenshots.
- Two RViz instances were running (user's interactive + the detached one); removed the duplicate.
- WARN: root link `base_link` has inertia; KDL ignores it (harmless for RViz; matters for Gazebo — Q-008).
Tooling only; no repo files changed.

---
## 2026-06-09 — VNC path for remote display (Route B)
Route A (ssh -Y to Mac) failed: Mac `$DISPLAY` empty (XQuartz not serving), so chose Route B.
Probe: desktop session is X11 + active; Xorg `-auth /run/user/1000/gdm/Xauthority` (owned by `barq`,
so no root needed to run x11vnc); port 5900 free; passwordless sudo NOT available (install must be
user-run). Added `~/setup_vnc.sh` (host): installs x11vnc via sudo, sets a VNC password, `xhost
+SI:localuser:root` so the container can draw on :0, starts `x11vnc -display :0` on :5900 (-bg).
Tightened `run_barq_gui.sh` to use the gdm Xauthority when `DISPLAY=:0`. View from Mac:
`vnc://barq.local:5900`. No repo files changed.

---
## 2026-06-09 — Remote display tooling (run_barq_gui.sh)
Added `~/run_barq_gui.sh` (host, alongside `run_barq.sh`): same container launch but with X11
passthrough — DISPLAY + wildcard `xauth` cookie merge + `/tmp/.X11-unix` mount + `QT_X11_NO_MITSHM`,
and `LIBGL_ALWAYS_INDIRECT` auto-set (direct for local `:0`, indirect for forwarded). Lets RViz/GUIs
show on the Mac via SSH X11 forward (Route A) or render on the Jetson `:0` GPU for VNC (Route B).
Host facts confirmed: live Xorg on `:0`, sshd `X11Forwarding yes`, `xauth` installed. No repo files changed.

---
## 2026-06-09 — Stage 2A wired up; URDF integrated & verified
Received artifacts validated and the visualization path made to actually work.

**Received (via barq-channel from the Mac):**
- `barq_description/urdf/barq.urdf` — 13 links, 12 revolute joints.
- `barq_description/meshes/{base_link,coxa,mid,mid_rev,foot,foot_rev}.dae` — all non-zero,
  `mid`/`mid_rev` and `foot`/`foot_rev` confirmed distinct (genuine mirrors).

**Changed:**
- `urdf/barq.urdf`: 24 mesh refs `filename="X.dae"` -> `package://barq_description/meshes/X.dae` (D-002).
- `barq_description/CMakeLists.txt`: added `install(DIRECTORY urdf meshes config ...)` (D-003).
- `config/robot_params.yaml`: placeholder geometry -> real values from URDF (D-005).
- `barq_bringup/launch/visualize.launch.py`: new — robot_state_publisher + joint_state_publisher_gui
  + rviz2, `gui:=true|false` arg (D-004).
- `barq_bringup/rviz/barq.rviz`: new — Grid + RobotModel(/robot_description) + TF, fixed frame base_link.
- `barq_bringup/setup.py`: install `launch/` and `rviz/`.
- `barq_bringup/package.xml`: added exec_depends (barq_description, robot_state_publisher,
  joint_state_publisher_gui, rviz2, xacro, launch, launch_ros).
- `.gitignore`: ignore `build/ install/ log/`.
- `docs/`: created the support-docs system (this folder).

**Verified (headless, inside `barq:dev`):**
- `check_urdf` — valid tree (base_link -> 4x coxa->femur->tibia).
- `colcon build --symlink-install` — 5/5 packages OK (only setuptools easy_install deprecation
  warnings on the Python packages).
- All 6 `package://` mesh targets exist in the installed share dir.
- `launch/visualize.launch.py` + `rviz/barq.rviz` present in installed share.
- `xacro` parse of the *installed* URDF — 13 links, 12 joints.

**Not yet verified:** actual RViz render (needs an X display).
**Not committed:** awaiting git policy (Q-003).
