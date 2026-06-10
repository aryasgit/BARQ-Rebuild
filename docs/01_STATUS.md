# BARQ — Current Status

**Last updated:** 2026-06-10
**Current stage:** 2D complete -> next 2E (physics sim)

## Snapshot
**BARQ walks in RViz from a velocity command.** The full kinematic pipeline is live and verified:
`/cmd_vel` -> trot gait -> `/foot_targets` -> IK -> `/joint_group_position_controller` -> joints -> RViz.

## Done
- [x] **2A** RViz visualization (`c04a3a8`)
- [x] **2B** ros2_control mock loop (`2d792aa`)
- [x] **2C** analytical IK (`938cab9`)
- [x] **2D** trot gait:
  - `gait.py` (trot foot-trajectory generator) + 6 unit tests; `gait_planner_node.py` (/cmd_vel -> /foot_targets)
  - `control.launch.py gait:=true` (control loop + IK + gait together)
  - Verified live: walking; diagonal pairs (FL+RR, FR+RL) in sync in `/joint_states`
  - **Foundation audit clean**; all 12 `barq_control` tests pass (incl. flake8 + pep257)

## Recent fixes (2026-06-10, after 2D)
- [x] **Leg fold corrected** (Q-010 -> D-009): IK knee-bend branch flipped to -1 — legs now fold
      FORWARD (knees +, ankles − in [-1.57, 0], matching the servo tibia range). Foot paths unchanged.
      13 tests pass. (A frame-flip attempt f4cd735 was reverted by request first.)
- [x] **Simulator decided** (Q-002 -> D-010): Gazebo for 2E, MuJoCo/Isaac for RL.
- [x] **Gait reversed + crouched** (D-011): cmd_vel +x now walks head-first (negated x,y mapping in
      gait_planner only); stand_height 0.16 / step_height 0.012 for a lower, stabler stance.
      14 tests pass (new: full-cycle tibia stays in [-1.571, 0]).

## Next
- [ ] **2E physics sim (Gazebo)** — add gz_ros2_control to the Dockerfile; same ROS interface, adds
      gravity + ground contact (body finally translates instead of stepping in place)
- [ ] parked: Q-001 tibia limit (evidence now favors servo's [-1.571,0]), Q-003 git push,
      Q-012 travel-direction vs robot front (deferred by Aryaman)
- [ ] gait tuning continues at 2E (step_height / stand_height vs the ankle limit)

## How to run
- **Walk:** `ros2 launch barq_bringup control.launch.py gait:=true`
  - `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"` (forward)
  - `"{angular: {z: 0.6}}"` (turn in place) · `"{linear: {y: 0.1}}"` (strafe)
- IK only: `... ik:=true` · Control loop: `control.launch.py` · Viz sliders: `visualize.launch.py`
- Tests: `cd src/barq_control && python3 -m pytest test/`
- Over VNC: `DISPLAY=:0 ~/run_barq_gui.sh` then `vnc://barq.local:5900`

## Interfaces
- `/cmd_vel` (geometry_msgs/Twist) — linear.x, linear.y, angular.z
- `/foot_targets` (Float64MultiArray[12]) — foot xyz, body frame, FL/FR/RL/RR
- `/joint_group_position_controller/commands` (Float64MultiArray[12]) — FL/FR/RL/RR x (coxa, femur, tibia)

## Notes
- RViz is kinematic-only (no ground/physics; body fixed) until 2E.
- `/joint_states` keyed by **name** (Q-005). Benign FIFO RT warning under Docker (Q-009).
- Gait defaults `stand_height=0.18`, `step_height=0.03` keep the ankle within +/-1.57 (legs need headroom to lift).
