# BARQ — Current Status

**Last updated:** 2026-06-10
**Current stage:** 2B complete -> starting 2C (IK)

## Snapshot
The full **ros2_control loop runs against mock hardware**: command 12 joint positions on a topic ->
they flow through controller_manager -> `/joint_states` -> TF -> RViz. Verified headless. This is the
exact machinery that will drive the real servos later (only the `<hardware>` plugin swaps).

## Done
- [x] **Stage 2A** — BARQ renders in RViz over VNC; 12 joints driveable. (commit `c04a3a8`, branch `stage-2`)
- [x] **Stage 2B** — ros2_control mock skeleton:
  - `barq.urdf.xacro` with a `<ros2_control>` block (`mode` = mock|gazebo|real; mock = `mock_components/GenericSystem`)
  - `ros2_controllers.yaml`: `joint_state_broadcaster` + `joint_group_position_controller` (all 12 joints)
  - `control.launch.py` (rsp + controller_manager + spawners + optional rviz)
  - Verified: 4 nodes up, both controllers ACTIVE, commanded pose appears in `/joint_states`.

## Next
- [ ] **Stage 2C — IK node**: analytical 3-DOF IK (coxa/femur/tibia), unit-tested, publishing to
      `/joint_group_position_controller/commands`; verify a foot target -> correct pose in RViz.
- [ ] Then 2D (gait), 2E (physics sim).
- [ ] Parked decisions: Q-001 tibia limit (**matters at 2C**), Q-002 simulator (2E), Q-003 git push.

## How to run
- **Viz only (GUI sliders):** `ros2 launch barq_bringup visualize.launch.py`
- **Full control loop:** `ros2 launch barq_bringup control.launch.py`
  - `ros2 control list_controllers`
  - `ros2 topic pub --once /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0,0.3,-0.6, 0,0.3,-0.6, 0,0.3,-0.6, 0,0.3,-0.6]}"`
- **Over VNC:** run it inside `DISPLAY=:0 ~/run_barq_gui.sh` (or a detached container), view `vnc://barq.local:5900`.

## Notes
- `/joint_states` is NOT in FL/FR/RL/RR order — always key by joint **name** (Q-005).
- ros2_control_node logs a benign "Could not enable FIFO RT scheduling" warning under Docker (Q-009).
- Headless Jetson display is 1024x768 (see memory `barq-remote-viz` / earlier changelog).
