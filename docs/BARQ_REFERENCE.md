# BARQ 2.0 — THE REFERENCE (the holy grail)

> One page of facts you can trust. Every number here is copied from its **owning file**, which is
> named next to it — if this sheet and the owning file disagree, the owning file wins and this
> sheet gets fixed in the same commit. For *where things live / what to read for a task*, see
> `MASTER_PROMPT.md`; for *every* tunable with provenance, `docs/roadmap/appendices/E_PARAMETER_REGISTRY.md`.
>
> Written 2026-09-30 (first session after the Jetson reflash), repo `stage-2` @ `a0ee1fc`.
> Marks: **[M]** measured · **[S]** spec/datasheet · **[J]** design judgment · **[TBD]** not yet known — never guess these.

---

## 1. Identity

| | |
|---|---|
| Project | BARQ 2.0 — 12-DOF quadruped, in-house design, final-year project |
| Team | Aryaman Gupta (git author for every commit, `rayman3304@gmail.com`) · Krish Agarwal |
| Repo | `github.com/aryasgit/BARQ-Rebuild`, working branch **`stage-2`** (main via PR) |
| Workspace | Jetson `~/barq_ws/` — repo is `~/barq_ws/src/`; `build/ install/ log/ artifacts/` sit outside the repo |
| Docs system | `docs/00–06` = what IS · `docs/roadmap/` = what's PLANNED · `docs/research/` = dated studies · `docs/reports/` = funder/prof reports |

## 2. Hardware (the bill that matters)

| Part | Fact | Source |
|---|---|---|
| Compute | Jetson Orin Nano devkit, 8 GB, 512 GB NVMe. **Reflashed Sep 2026 → JetPack 6.2, L4T R36.4.4** (container base expects r36 ✔) | `/etc/nv_tegra_release` [M] |
| Actuators | 12× Waveshare **ST3215** serial-bus servo: 30 kg·cm = **2.94 N·m** peak @12 V, **0.222 s/60° = 4.71 rad/s** no-load, 12-bit magnetic encoder (4096 counts/rev), 360° (no hard stops), half-duplex TTL bus, 1 Mbaud default | robot_params.yaml, D-018 [S] |
| Servo buses | 4 buses × 3 servos (one Waveshare driver board per leg) | 00_OVERVIEW, P1-03 |
| MCU | Teensy 4.1 — **status: debatable (see §12 Q1)**. Firmware (500 Hz superloop, protocol v1, deadman) is written and emulator-tested; servo/IMU/power stubs unfilled | barq_firmware/ |
| IMU | BNO085 (SH-2 rotation vector + gyro + accel) — planned | 06_PROTOCOL |
| Power monitor | INA260 (owned; integrated shunt, ±15 A/unit). Protocol text still says "INA226" (stale label; fields are chip-agnostic) | D-021 |
| Battery | GENX Premium **4S LiPo 5200 mAh, 512 g** (mass already inside the 1420 g body). Full ≈16.8 V (17.1–17.4 V if HV cells — verify chemistry before first charge) | D-021 |
| Servo rail | **4S NEVER goes to servos/driver boards directly** (ST3215 max 12.6 V) → mandatory high-current **12 V buck** (20–30 A class, sizing in P1-01) | D-021 |
| Jetson feed | Direct from 4S, fused (Orin Nano accepts 9–20 V) | D-021 |
| Brownout ladder | warn 14.0 V → fault bit2 13.8 V → controlled sit 13.6 V → firmware hard torque-off 13.2 V | D-021 [J, confirm on bench] |
| Lidar | Recommended **LDROBOT STL-27L** (~45 g, 25 m, 21.6 kHz); not purchased; sim twin already in URDF | research/2026-06-11-lidar-selection.md |

## 3. Geometry & mass (owning file: `barq_description/config/robot_params.yaml` + URDF)

| Item | Value | |
|---|---|---|
| Body box L×W×H | 0.258 × 0.117 × 0.085 m | URDF collision |
| Body mass | **1.42 kg** incl. electronics, fasteners, 512 g battery | [M] 2026-06-11 |
| Coxa / femur / tibia mass (each, servos incl.) | 0.0733 / 0.1536 / 0.030 kg | [M] |
| **Total robot** | **2.448 kg** (legs 1.028 kg) · sim model **2.495 kg** (+0.047 kg lidar) | [M] |
| Hip origins (x, y, z) from body centre | FL (+0.108484, +0.0171913, 0.00022) · FR (+0.108484, −0.0148092, 0.00022) · RL (−0.108371, +0.0167905, 0.00022) · RR (−0.108371, −0.01521, 0.00022) | L/R y asymmetric **by design** |
| Hip spacing | front↔rear 0.2169 m; stance foot width ≈ 0.183 m | derived |
| Exact leg chain (D-014) | knee_x ±0.01744 (+front/−rear) · knee_y 0.0430692 · ankle_x 0.018944 · ankle_y 0.0324 · femur_z 0.100 · lateral 0.0754692 · tibia 0.100 | locked to <1e-12 by `test_exact_kinematics.py` |
| Legacy idealized lengths | coxa 0.0465 · femur 0.107 — **NOT for control** (3.4 cm error) | D-014 |
| Foot | contact sphere r = 0.012 m at tibia tip; μ default 0.9 | URDF |
| Centre of mass | **[TBD]** — currently geometric centre (0,0,0); measure with battery installed | Q-014 |

## 4. Conventions (break these and things silently go wrong)

- **Frames: REP-103** — X forward, Y left, Z up. **Forward = body +X** (the FL/FR end). `forward_sign = +1`. Judge direction in physics, never in pinned-body RViz (D-015).
- **Leg order FL, FR, RL, RR**; joints per leg **coxa, femur, tibia**. URDF names: `<LEG>_hip_joint` (coxa, axis X), `<LEG>_knee_joint` (femur, axis Y), `<LEG>_ankle_joint` (tibia, axis Y). URDF *declares* legs FL, RL, FR, RR — always access by name, never by index (Q-005).
- **Servo IDs 0–11** = FL/FR/RL/RR × coxa/femur/tibia (FL_coxa=0 … RR_tibia=11).
- **Mirroring absorbed at the hardware layer** (D-001): all math is symmetric; `direction` = +1 everywhere except FR_coxa, RR_coxa = −1. `zero_offset` all **[TBD]** (filled at P2-03 calibration).
- **IK knee branch = −1** (legs fold forward, D-009).
- **Joint limits [J]** (360° servos have no stops, D-012): hip ±0.785 · knee ±1.571 · **ankle [−2.2, 0]** (URDF upper is +1.57 — unused headroom; IK clamps at 0). Check physical collision at −2.2 before driving real servos there.
- Units: SI in ROS; protocol uses mrad / 10 mrad·s⁻¹ / 0.1 % / ×1e-4 / cm·s⁻² / mV / mA.

## 5. Stance & gait (owning files: `gait_planner_node.py`, `ik_node.py`)

| Param | Value | Why |
|---|---|---|
| stand_height | 0.13 m | D-014 |
| rear_raise | 0.02 m → body ≈ −4.5° nose-down **by design** | D-016 load-forward |
| step_height | 0.02 m (front clearance is reach-capped ~20 mm) | D-019 |
| period / duty | 0.5 s / **0.6** (0.55 = dead-straight open-loop) | D-019, Q-016 |
| Swing profile | smoothstep horizontal, ~zero-velocity touchdown | D-019 |
| Stance joint init (hip, knee, ankle) | front (0, 1.047531, −1.928768) · rear (0, 0.911998, −1.652637) | URDF initial_value |
| Settle height (base_link z) | 0.1418 m measured vs 0.142 predicted | research log |
| Tibia fold reach floor | 0.1079 m in-plane at q3 = −2.2 | D-019 |

## 6. Rates & safety layers

gait 50 Hz → ik 50 Hz → controller_manager / CMD / STATE **100 Hz** → Teensy superloop **500 Hz**.
Deadmen: gait cmd_vel **1 s** → hw-interface stale link **300 ms** → firmware **200 ms** (torque off, bit3). **Killing the gait node does NOT torque-off** — see P4-03 stop ladder.

## 7. Protocol v1 (Jetson ↔ Teensy, `docs/06_PROTOCOL.md`)

Frame `BA 51 | ver 01 | type | seq | len | payload | CRC16-CCITT-FALSE(0x1021, init 0xFFFF) over ver..payload`, little-endian, len ≤ 200, resync decoder.
CMD 0x01: 12×int16 mrad (24 B). STATE 0x02 @100 Hz: 98 B (pos, vel, load, quat, gyro, accel, vbus, current, temp_max, fault). PING 0x03 / PONG 0x83. Fault bits: 0 servo-bus, 1 IMU stale, 2 power, 3 deadman. **Change bytes only with golden vectors regenerated in C++ AND Python in one commit.**

## 8. Simulation facts (Gazebo Fortress via `ign_ros2_control`)

| Item | Value |
|---|---|
| Engine envelope | effort 2.94 N·m, velocity 4.71 rad/s on all 12 (verified engine-side, `ign sdf -p`) |
| Servo stiffness | `position_proportional_gain: 0.6` (k = 60 s⁻¹) in `ros2_controllers.yaml` — **only works with the vendored patched plugin** `external/gz_ros2_control`; canary: reads 0.1 = soft fallback |
| Joint dynamics | damping 0.05, friction 0 |
| Spawn | z = 0.17 m |
| Worlds | `barq_sim/worlds/barq_world.sdf` (8×6 m room + pillars), `barq_course.sdf` (10×8 m: doorway, slalom, boxes) |
| Sensors | lidar 2160 samples @10 Hz, 0.15–25 m, σ 0.02, mount (−0.04, 0, 0.066) pitched −4.5°; IMU 100 Hz on base_link → `/imu/data` |
| Effort readout | effort state interface (gazebo mode only) → `/joint_states.effort` = transmitted joint torque incl. ground reaction |

## 9. Baselines — do not regress (research log §1, §2f, §2g)

| Metric | Value |
|---|---|
| Walk @ vx 0.15, duty 0.6 | **≈60 % of commanded** (50–70 % pass band); duty 0.55 → 47 %, straight |
| Joint tracking | 17.8 mrad mean RMS (k=60) |
| Step response | 50 ms rise (theory 51 ms), no overshoot |
| Legged odometry drift | 4–5 % of distance |
| Torque, normal trot | worst RMS 1.31 N·m (RL ankle, 45 % cap), sustained peak 1.86 N·m (63 %), impact transients ≈ cap; **continuous safety factor 2.2×**; rear legs carry ~2× front (Q-017) |
| Autonomy | nav2 mission success (0.14 m error); obstacle course 16 m completed with self-recoveries |
| Tests | pytest 30 pass + 1 skip · `pio test -e native` 6/6 · `integration_pty.py` 9/9 |

## 10. Paths & commands

| What | Where |
|---|---|
| Robot model | `barq_description/urdf/barq.urdf.xacro` (`mode:=mock\|gazebo\|real`, `device`, `foot_mu`) |
| Single source of truth | `barq_description/config/robot_params.yaml` |
| Control nodes | `barq_control/barq_control/{leg_kinematics,gait,gait_planner_node,ik_node,state_estimator_node,barq_protocol}.py` |
| Launch | `barq_bringup/launch/sim.launch.py` (args: world_file gui gait slam nav odom_source foot_mu gait_duty gait_period) · `real.launch.py device:=` |
| HW interface | `barq_hw/` (BarqSystem + teensy_emulator + integration_pty.py) |
| Firmware | `barq_firmware/` (PlatformIO; COLCON_IGNOREd; build on HOST, `~/.local/bin/pio`) |
| Diagnostics | `diagnostics/` (st3215_diag, sim_walk_metric, sim_actuation_probe, analyze_track_bag, torque_from_bag, plot_torque) |
| Demo scripts (untracked) | `~/barq_ws/_tour.py` (nav2 grand tour), `_wave.py` (dance), `_squat.py` (IK squat) |
| Setup (post-reflash) | `~/barq_setup/jetson_bootstrap.sh` (sudo: groups, x11vnc service, virtual 1080p monitor) |

```bash
# image + workspace (inside barq:dev)
docker build -t barq:dev ~/barq_ws/src
docker run --runtime nvidia -it --rm --network host --shm-size=8g -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev
GZ_VERSION=fortress colcon build --packages-select gz_ros2_control      # FIRST, always
colcon build --symlink-install --packages-skip gz_ros2_control && source install/setup.bash
ros2 launch barq_bringup sim.launch.py gait:=true gui:=true            # + slam:=true nav:=true
```

## 11. Environment facts (post-reflash, 2026-09-30)

- Hostname **`barq-desktop`** → mDNS `barq-desktop.local` (older docs say `barq.local`). Wi-Fi IP at time of writing 172.18.60.151 (DHCP — may change).
- Mac (MacBook Pro M1 Pro) is the SSH client; Jetson is headless, GNOME on Xorg `:0`, autologin `barq`, Wayland disabled.
- Remote screen: x11vnc service `barq-vnc` on :5900 → Mac Finder ⌘K `vnc://barq-desktop.local`.
- GitHub over **SSH port 443** (`~/.ssh/config`); new Jetson key `barq-jetson` must be added to GitHub.
- Hard rules: one ROS stack at a time · `-v /dev/shm:/dev/shm` on every container · `timeout -k 2 N ros2 …` always · no robot-side GUIs during nav missions (Orin saturates) · trust VNC not screenshots for GL.

## 12. Open items (live)

| ID | Item |
|---|---|
| Q-014 | Exact CoM (battery installed) → base_link inertial origin; may shrink rear_raise |
| Q-016 | Open-loop yaw vs duty (fix: estimator yaw-rate feedback into gait wz) |
| Q-017 | Rear-ankle torque headroom cost of D-016 trim |
| Q-009 | RT scheduling in Docker for the real control loop |
| Q-015 | Lidar purchase (STL-27L) |
| Q1 (new) | Teensy in or out? (vs Jetson → Waveshare USB bus adapters directly) |
| Q2 (new) | Hardware progress since June (INA260 was probed on Jetson I2C bus 7 @ 0x40) |
