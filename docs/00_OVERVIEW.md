# BARQ — Project Overview

> Durable reference. Distilled from the kickoff doc and verified against the URDF on 2026-06-09.

## Mission
BARQ is a 12-servo quadruped robot. Goal: walk under a velocity command, first in
visualization, then physics sim, then on real hardware, and eventually under a learned (RL) policy.
Authors: Aryaman Gupta + Krish Agarwal.

## Hardware topology
```
12x ST3215 servos (4 UART buses, 3/bus)
   <-> Teensy 4.1  (bare-metal 500Hz superloop: sync-write targets, read servo state,
                    BNO085 IMU [SH-2: quat+gyro+accel], INA226 power, safety watchdog,
                    custom binary protocol over USB serial)
   <-> Jetson Orin Nano (Docker, ROS 2 Humble)  <-- WE ARE HERE
```
The Jetson runs everything in the `barq:dev` Docker image (ROS 2 Humble desktop, L4T r36.4.0 /
JetPack 6). Claude Code tools run *on the Jetson host*; the Mac Mini is the SSH client and pushes
files via the `~/.barq-channel.sh` rsync helpers (`barq-urdf`, `barq-meshes`, ...).

## Control architecture (ROS 2 graph)
```
barq_hardware (C++ ros2_control)  <-> Teensy serial   |  publishes /imu/data, /power/data
  -> JointStateBroadcaster -> /joint_states
state_estimator_node.py : /joint_states + /imu/data        -> /robot_state
gait_planner_node.py    : /robot_state + /cmd_vel          -> /foot_targets
ik_node.py              : /foot_targets                     -> /joint_targets
[future] rl_policy_node.py : /robot_state + /cmd_vel        -> /joint_targets (replaces gait+IK)
```
**Principles**
- Python everywhere except the C++ hardware interface and the Teensy firmware.
- Mirroring absorbed at the hardware layer (direction multipliers in `robot_params.yaml`); all
  higher layers work in a symmetric frame. See `config/joint_conventions.yaml` (Option A).
- Single source of truth: all geometry & conventions in `barq_description/config/` + `urdf/`.
- **Mock -> Sim -> Real**: only the hardware interface changes; everything above is identical.
- RL trains off-board (x86 + RTX, Isaac Lab), deploys as ONNX -> TensorRT on the Jetson.

## Frame conventions (REP-103)
X forward, Y left, Z up. Legs: `[FL, FR, RL, RR]`. Joints per leg: `[coxa, femur, tibia]`.
Axes: coxa about X (abduction), femur about Y, tibia about Y.

## Verified geometry (from `urdf/barq.urdf`, 2026-06-09)
Kinematic tree confirmed via `check_urdf`: `base_link` -> 4 legs, each `coxa -> femur -> tibia`
(13 links, 12 revolute joints).

| Item | Value | Source |
|---|---|---|
| Body collision box (LxWxH) | 0.258 x 0.117 x 0.085 m | URDF base_link |
| Body mass | 0.95 kg (total incl. legs ~1.66 kg) | URDF inertials |
| Coxa length | 0.0465 m | sqrt(0.01744^2 + 0.0430692^2) |
| Femur length | 0.107 m | sqrt(0.018944^2 + 0.0324^2 + 0.1^2) |
| Tibia length | 0.100 m | ankle->foot Z |
| Coxa (hip) limit | +/-0.785 rad (+/-45 deg) | URDF |
| Femur (knee) limit | +/-1.57 rad (+/-90 deg) | URDF |
| Tibia (ankle) limit | +/-1.57 rad in URDF (see Q-001 conflict) | URDF |

Hip joint origins from body centre (exact, mirrored into `robot_params.yaml`):
| Joint | X | Y | Z |
|---|---|---|---|
| FL | +0.108484 | +0.0171913 | +0.00022012 |
| FR | +0.108484 | -0.0148092 | +0.00022176 |
| RL | -0.108371 | +0.0167905 | +0.00022176 |
| RR | -0.108371 | -0.01521   | +0.00022012 |
> L/R Y offsets are intentionally asymmetric (~17 mm L vs ~15 mm R). Hip Z ~ 0.

URDF<->convention name map: `<LEG>_hip_joint`=coxa (axis X), `<LEG>_knee_joint`=femur (axis Y),
`<LEG>_ankle_joint`=tibia (axis Y).

Meshes (in `barq_description/meshes/`, referenced via `package://`):
`base_link.dae` (body); `coxa.dae` (all 4 coxae); `mid.dae`/`mid_rev.dae` (L/R femur);
`foot.dae`/`foot_rev.dae` (L/R tibia). `_rev` = mirrored for the right side.

## Package structure
```
barq_ws/src/
  barq_description/  single source of truth: urdf/, meshes/, config/  (no nodes; ament_cmake)
  barq_hardware/     C++ ros2_control hardware interface (Teensy comms)  (ament_cmake)
  barq_control/      Python nodes: state_estimator, gait_planner, ik, [rl_policy]  (ament_python)
  barq_sim/          physics sim launch + worlds  (ament_python)
  barq_bringup/      top-level launch files + rviz config  (ament_python)
  docs/              this folder
```

## Staged plan
- **2A URDF + Visualization** *(current)* — see BARQ in RViz, joints driveable from GUI sliders.
- **2B ros2_control mock skeleton** — full control loop vs `mock_components`; `/joint_states` flows.
- **2C IK node** — 3-DOF analytical IK from link lengths; verified in RViz.
- **2D gait planner** — trot gait; cmd_vel -> gait -> IK -> joints -> RViz walk.
- **2E physics sim** — same ROS interface; tune gait against physics. (Engine: see Q-002.)
- **3 Teensy firmware** — BNO085 + INA226 + binary protocol + watchdog + ST3215 bus.
- **4 real hardware interface** — replace mock with real Teensy serial; robot moves.
- **5 RL** — train PPO (Isaac Lab) + domain randomization; deploy ONNX->TensorRT as a ROS node.

## Build / run (inside `barq:dev`)
```
~/run_barq.sh                       # enter dev container (mounts ~/barq_ws)
colcon build --symlink-install
source install/setup.bash
ros2 launch barq_bringup visualize.launch.py     # Stage 2A (needs an X display)
```
