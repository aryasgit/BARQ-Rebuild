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
  - Physics verified: settle z 0.1418 (predicted 0.142), walk cmd +x -> head-first (-X),
    19 unit tests green

## Next (pick)
- [ ] **Stage 3 — Teensy firmware** (parallel track; BNO085 + INA226 in hand)
- [ ] Gait quality in sim: realized speed ~40% of command (Q-013) — tune period/friction/steps
- [ ] State estimator node (needs IMU — sim can provide one earlier if useful)
- [ ] Q-012 leg-label rename (decide against CAD before Stage 4)

## How to run
- **Physics sim (walks for real):** `ros2 launch barq_bringup sim.launch.py gait:=true gui:=true`
  - drive: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}"`
  - headless verify: `gui:=false`; pose: `ign model -m barq --pose`
- Kinematic-only RViz: `control.launch.py gait:=true` · sliders: `visualize.launch.py`
- Tests: `cd src/barq_control && python3 -m pytest test/`
- Over VNC: container needs the X mounts (see `~/run_barq_gui.sh`); view `vnc://barq.local:5900`
- **One stack at a time** — host-network containers share the DDS graph (cross-talk!).

## Interfaces (unchanged)
`/cmd_vel` (Twist, +x = head-first) · `/foot_targets` (12 xyz, body frame, FL/FR/RL/RR)
· `/joint_group_position_controller/commands` (12 rad, FL/FR/RL/RR x coxa,femur,tibia)
