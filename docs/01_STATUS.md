# BARQ — Current Status

**Last updated:** 2026-06-10
**Current stage:** 2C complete -> starting 2D (gait)

## Snapshot
**Analytical IK works end-to-end**: give the IK node a foot position (body frame) and it computes the
joint angles that place the foot there, streaming to the control loop -> RViz. Verified by unit tests
(FK<->IK round-trip to 1e-9) and live (stance + foot-target response).

## Done
- [x] **2A** — RViz visualization, 12 joints driveable. (`c04a3a8`)
- [x] **2B** — ros2_control mock loop. (`2d792aa`)
- [x] **2C** — analytical IK:
  - `leg_kinematics.py` — idealized 3-DOF FK + analytical IK; unit-tested round-trip to 1e-9 (4 tests)
  - `ik_node.py` — reads robot_params geometry; streams default stance; `/foot_targets` -> 12 joint cmds @ 50 Hz
  - `control.launch.py ik:=true` brings up control loop + IK together
  - Verified live: stance (hips 0, knees -0.73, ankles 1.52); feet 0.19 m below hips -> legs straighten

## Next
- [ ] **2D — gait planner**: `/cmd_vel` -> foot trajectories (trot) -> `/foot_targets` -> (IK) -> walk in RViz
- [ ] then 2E physics sim
- [ ] parked: Q-001 tibia limit, Q-002 simulator, Q-003 git push, Q-010 knee-bend visual check

## How to run
- Viz (sliders):       `ros2 launch barq_bringup visualize.launch.py`
- Control loop:        `ros2 launch barq_bringup control.launch.py`
- Control loop + IK:   `ros2 launch barq_bringup control.launch.py ik:=true`
  - move feet: `ros2 topic pub --once /foot_targets std_msgs/msg/Float64MultiArray "{data: [FLx,FLy,FLz, FRx,FRy,FRz, RLx,RLy,RLz, RRx,RRy,RRz]}"`
- IK unit tests:       `cd src/barq_control && python3 -m pytest test/test_ik.py`
- Over VNC:            `DISPLAY=:0 ~/run_barq_gui.sh` then `vnc://barq.local:5900`

## Interfaces
- `/foot_targets` (Float64MultiArray[12]) — foot xyz in **body frame**, order FL, FR, RL, RR
- `/joint_group_position_controller/commands` (Float64MultiArray[12]) — joint pos, FL/FR/RL/RR x (coxa, femur, tibia)

## Notes
- RViz is kinematic-only (no ground/physics; body fixed) until 2E.
- `/joint_states` not in FL/FR/RL/RR order — key by **name** (Q-005).
- Benign FIFO RT scheduling warning under Docker (Q-009).
