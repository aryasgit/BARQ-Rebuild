# BARQ

12-servo quadruped robot — **Aryaman Gupta + Krish Agarwal**.

12× Waveshare ST3215 (4 serial buses) · Teensy 4.1 (500 Hz firmware, custom binary protocol
over USB) · Jetson Orin Nano (ROS 2 Humble in Docker) · BNO085 IMU · INA260 power telemetry ·
4S LiPo with a 12 V buck servo rail · STL-27L-class lidar (sim model live, unit pending).

**Status (2026-06-13)** — the entire software stack is proven in simulation and against
emulated firmware; physical parts are arriving:

- **Walks in Gazebo physics** with the actuator model calibrated to the real servo
  (engine-verified 2.94 N·m / 4.71 rad/s caps + ST3215-class stiffness via a vendored
  plugin patch).
- **Maps and navigates autonomously** (slam_toolbox + nav2): a 16 m unknown obstacle course
  completed with self-recoveries, running on honest legged odometry (~4–5 % drift), not
  ground truth.
- **Stage-4 hardware interface passes 9/9 integration checks** against the *real firmware
  logic* on an emulated Teensy (PTY). Hardware day is literally `device:=/dev/ttyACM0`.
- **A 33-document execution plan** (`docs/roadmap/`) covers everything from bench calibration
  to RL deployment — written to be executable without LLM assistance.

## Start here

| You are… | Read |
|---|---|
| new to the repo (human **or** LLM) | **[`MASTER_PROMPT.md`](MASTER_PROMPT.md)** — the complete index + task routing table |
| resuming work | [`docs/HANDOFF.md`](docs/HANDOFF.md) → [`docs/01_STATUS.md`](docs/01_STATUS.md) |
| building the physical robot | [`docs/roadmap/README.md`](docs/roadmap/README.md) → phase folders P0–P7 |
| looking for any command | [`docs/roadmap/appendices/C_COMMAND_CRIB.md`](docs/roadmap/appendices/C_COMMAND_CRIB.md) |
| auditing a decision | [`docs/02_DECISIONS.md`](docs/02_DECISIONS.md) (D-000…D-022) + [`docs/05_RESEARCH_LOG.md`](docs/05_RESEARCH_LOG.md) |

## Packages

| Directory | Role |
|---|---|
| `barq_description` | URDF/xacro (exact measured masses, `mode:=mock\|gazebo\|real`), meshes, `robot_params.yaml` (single source of truth) |
| `barq_control` | trot gait, exact analytical IK, legged-odometry estimator, protocol codec — plus the 30-test suite |
| `barq_bringup` | launch files (`sim` / `real` / RViz-only) + controller, SLAM, nav2 configs |
| `barq_hw` | C++ ros2_control hardware interface + the Teensy **emulator** (real firmware logic on a PTY) |
| `barq_firmware` | Teensy 4.1 PlatformIO project; `loop_core.{h,cpp}` is shared verbatim with the emulator |
| `barq_sim` | Gazebo Fortress worlds (walled room, obstacle course) |
| `diagnostics` | ST3215 bench tool + sim-fidelity probes (step response, walk metric, bag analyzer) |
| `external/gz_ros2_control` | vendored, 3-line-patched sim plugin — **build once per fresh clone** (see roadmap P0-01) |
| `docs` | the documentation system: status, decisions, changelog, open questions, research log, protocol spec, roadmap |

## Quick start — simulation (on the Jetson)

```bash
docker run -d --rm --name barq_sim --network host \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev \
  bash -lc 'source /opt/ros/humble/setup.bash; cd /root/barq_ws; source install/setup.bash; \
            exec ros2 launch barq_bringup sim.launch.py gait:=true slam:=true nav:=true'

# drive it
docker exec barq_sim bash -lc 'source /opt/ros/humble/setup.bash; source /root/barq_ws/install/setup.bash; \
  timeout -k 2 30 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}"'
```

Fresh clone? Follow `docs/roadmap/phase-0-environment/01_REBUILD_FROM_ZERO.md` (image build,
workspace build **including the `external/` overlay**, validation gates).

## Quick start — robot stack with zero hardware

```bash
ros2 run barq_hw teensy_emulator                       # prints: PTY /dev/pts/N
ros2 launch barq_bringup real.launch.py device:=/dev/pts/N gait:=true
ros2 run barq_hw integration_pty.py                    # the 9-check Stage-4 contract
```

The same `real.launch.py` line, with the device swapped, is the robot bringup.

## Tests (the regression spine)

```bash
cd src/barq_control && python3 -m pytest test/   # 30 passed
cd src/barq_firmware && pio test -e native       # 6/6 (host-compiled Unity)
ros2 run barq_hw integration_pty.py              # 9/9 end-to-end
```

Full spine + per-phase acceptance gates: `docs/roadmap/appendices/B_ACCEPTANCE_GATES.md`.
