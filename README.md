# BARQ
12-servo quadruped robot. Aryaman Gupta + Krish Agarwal.

## Package structure
- `barq_description` — URDF, joint conventions, robot params (single source of truth)
- `barq_hardware` — ros2_control hardware interface (Teensy comms)
- `barq_control` — gait, IK, state estimation nodes
- `barq_sim` — MuJoCo/Gazebo launch and world files
- `barq_bringup` — top-level launch files

## Quick start
- Enter dev container: `~/run_barq.sh`
- Build: `colcon build --symlink-install`
- Source: `source install/setup.bash`
