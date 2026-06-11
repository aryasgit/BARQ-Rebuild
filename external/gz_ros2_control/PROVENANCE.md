# Vendored: gz_ros2_control (BARQ-patched)

- Origin: https://github.com/ros-controls/gz_ros2_control — branch `humble`,
  commit `b09ae4cc5d6ab19a5c61cca6d9cc6dfb2ca4ee71` (v0.7.20), vendored 2026-06-11.
- Why vendored: upstream 0.7.x constructs the plugin's rclcpp node BEFORE installing the
  `<parameters>` yaml into the rcl context's global arguments
  (gz_ros2_control/src/gz_ros2_control_plugin.cpp, Configure()). Node parameter overrides are
  resolved at node construction, so `position_proportional_gain` — the sim servo stiffness,
  the single most transfer-critical actuation number — silently could NOT be configured and
  stuck at the default 0.1 (k = gain x update_rate = 10/s; the BARQ trot needs ~60/s to match
  an ST3215-class internal loop; see docs/05_RESEARCH_LOG.md and D-018).
- The fix: `BARQ.patch` (3 lines) — pass the already-collected `--ros-args --params-file ...`
  vector into the node's own NodeOptions. Upstream-relevant; consider a PR.
- Only the `gz_ros2_control` package builds (provides `libign_ros2_control-system.so`, which
  `barq_bringup/launch/sim.launch.py` puts FIRST on IGN_GAZEBO_SYSTEM_PLUGIN_PATH so it
  shadows the /opt binary). Demos/tests are COLCON_IGNOREd.
- Rebuild: `GZ_VERSION=fortress colcon build --packages-select gz_ros2_control`
  (needs libignition-gazebo6-dev — already in the barq:dev image).
