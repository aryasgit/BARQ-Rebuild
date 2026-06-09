# BARQ — Current Status

**Last updated:** 2026-06-09
**Current stage:** 2A complete -> starting 2B

## Snapshot
**BARQ renders correctly in RViz**, viewed from the Mac over VNC. All meshes load, TF publishes,
and the 12 joints are driveable from the joint_state_publisher_gui sliders. Stage 2A is done.

## Done
- [x] 5-package workspace; URDF (13 links / 12 joints, valid tree) + 6 meshes received & verified.
- [x] URDF mesh refs -> `package://`; barq_description installs urdf/meshes/config; robot_params updated.
- [x] `visualize.launch.py` + `barq.rviz`; `colcon build` green (5/5).
- [x] Remote display solved: Jetson is headless; forced `:0` to a real mode (xorg.conf ConnectedMonitor
      on DP-0) + x11vnc on `:0`, viewed via macOS Screen Sharing (`vnc://barq.local:5900`).
- [x] **RViz visual check PASSED** — body + 4 legs + feet render, geometry looks correct, GPU GL 4.6.

## Next
- [ ] Stage 2B — ros2_control mock skeleton (ros2_control.yaml + controller_manager + JointStateBroadcaster).
- [ ] Optional: bump VNC desktop 1024x768 -> 1600x900 (needs a real EDID blob; see CHANGELOG).
- [ ] Resolve open questions Q-001..Q-003 (tibia limit, simulator, git policy) when convenient.

## How to view RViz (current working setup)
1. Jetson: VNC server runs on `:0` (restart with `~/setup_vnc.sh` if it ever drops).
2. RViz runs in a detached container `barq_rviz` (stop: `docker stop barq_rviz`). Or run it interactively:
   `DISPLAY=:0 ~/run_barq_gui.sh` then `ros2 launch barq_bringup visualize.launch.py`.
3. Mac: Finder -> Cmd-K -> `vnc://barq.local:5900` (or `vnc://10.79.88.160:5900`).

## Display facts (headless Jetson)
- Orin DP-0/DP-1 disconnected; NVIDIA Tegra driver. `~/fix_display.sh` forces DP-0 connected at a mode.
- Currently **1024x768** (driver's default mode pool without EDID). 1600x900 needs an EDID blob.
- Host screenshots of direct-GL windows come back black — verify via VNC, not host screenshots.
