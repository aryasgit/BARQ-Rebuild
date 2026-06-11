# BARQ — Current Status

**Last updated:** 2026-06-11
**Current stage:** 2E complete — BARQ walks in Gazebo physics

## Snapshot
**The full kinematic + physics pipeline works end-to-end**: `/cmd_vel` -> trot gait -> exact IK ->
ros2_control -> Gazebo Fortress (gravity + ground contact) -> the body actually translates,
head-first, level, straight. Settle height matches prediction to 0.2 mm.

## Done
- [x] **2A** RViz viz (`c04a3a8`) · **2B** ros2_control mock (`2d792aa`) · **2C** IK (`938cab9`)
      · **2D** trot gait (`2982b06`) + fold/direction/crouch fixes (`1affccf`..`276b67d`)
- [x] **2E** Gazebo Fortress sim:
  - barq:dev image + slim ros-gz set (OpenCV-conflict workaround for dustynv base)
  - mode:=gazebo (ign_ros2_control), foot-sphere contacts, offline world, `sim.launch.py`
  - 25-agent adversarial review -> 3 confirmed findings, all fixed
  - **Exact URDF-true kinematics (D-014)** replacing the 3.4 cm-error idealized model,
    locked by chain-composition tests; stance initial_values kill the startup snap
  - Physics verified: settle z 0.1418 (predicted 0.142); walk cmd +x -> **+X = forward**
    (D-015, ruled by Aryaman against the physics walk; forward_sign=+1); 19 unit tests green

## Direction (Aryaman 2026-06-11): SIM-TO-THE-MAX until parts arrive
Everything tunable in sim first (offsets, balance, stability); RL outlook improved by sim fidelity.
Bench tooling ready in `diagnostics/` for the day servos arrive.

## Next (pick)
- [x] **Stage 3 opened**: Jetson<->Teensy protocol test-first on both ends (golden-vector pinned);
      loopback firmware v0 compiles for teensy41 — flash-day ready (docs/06_PROTOCOL.md)
- [x] **Stage 4 interface DONE (D-020)**: barq_hw/BarqSystem + teensy_emulator (real LoopCore
      on a PTY) — integration 9/9, full-gait rehearsal on the emulator at 100 Hz.
      Drop-in = flash + fill servo/IMU/power stubs + device:=/dev/ttyACM0 (real.launch.py)
- [x] **Sim actuation made ST3215-true (D-018)**: velocity caps 4.71, k=60/s servo stiffness
      (vendored plugin patch — external/gz_ros2_control), tracking 17.8 mrad RMS, friction
      parameterized + swept
- [x] **Q-013 solved (D-019)**: speed deficit was swing-foot drag (mu-invariant fingerprint);
      smoothstep swing + duty 0.6 -> ~60% realized (0.55 = dead straight, 47%; Q-016 tracks yaw)
- [x] **Sim perception**: lidar (STL-27L-class) + SLAM mapping the 8x6 room in Gazebo (slam:=true)
- [x] **Autonomy**: nav2 mission SUCCEEDED — click-to-navigate via RViz "2D Goal Pose" (nav:=true)
- [x] **Obstacle course COMPLETED**: 16 m, doorway+slalom+box field, autonomous spin/wait recoveries,
      dynamic confidence-regulated speed (0.22 ceiling) — see RESEARCH_LOG 2d
- [x] **State estimator v1**: sim IMU + stance-diagonal legged odometry, drift ~4-5%; SLAM validated
      on honest odometry (`odom_source:=estimated`) — last ground-truth seam closed (D-017)
- [x] ~~Q-012 leg-label rename~~ — resolved: labels match quadrants (D-015)

## How to run
- **Physics sim (walks for real):** `ros2 launch barq_bringup sim.launch.py gait:=true gui:=true`
  - drive: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}"`
  - sim-fidelity knobs: `foot_mu:=0.5 gait_duty:=0.55 gait_period:=0.5`; metrics:
    `diagnostics/sim_walk_metric.py`, `sim_actuation_probe.py step`, bag + `analyze_track_bag.py`
  - NOTE fresh clones: build `gz_ros2_control` once (`GZ_VERSION=fortress colcon build
    --packages-select gz_ros2_control`) or the sim silently falls back to the soft /opt plugin
  - headless verify: `gui:=false`; pose: `ign model -m barq --pose`
- Kinematic-only RViz: `control.launch.py gait:=true` · sliders: `visualize.launch.py`
- Tests: `cd src/barq_control && python3 -m pytest test/`
- Over VNC: container needs the X mounts (see `~/run_barq_gui.sh`); view `vnc://barq.local:5900`
- **One stack at a time** — host-network containers share the DDS graph (cross-talk!).

## Interfaces (unchanged)
`/cmd_vel` (Twist, +x = forward = body +X) · `/foot_targets` (12 xyz, body frame, FL/FR/RL/RR)
· `/joint_group_position_controller/commands` (12 rad, FL/FR/RL/RR x coxa,femur,tibia)
