# Lidar selection research — RPLidar A2M12 vs alternatives (2026-06-11)

> Produced by a research subagent (web-verified specs + live checks on the BARQ Jetson).
> Decision: Q-015. Kept verbatim for the publication record; sim-integration plan in §5.

**TL;DR verdict:** The A2M12 works and has the most mature ROS 2 driver story, but at 190 g / 76 mm
diameter it is the wrong mass class for a 2.45 kg robot — it adds ~9% body mass at the worst
possible place (high on the torso) for ~$200–230, and 2D at this height misses most of what BARQ
actually needs to avoid. **Recommended instead: LDROBOT STL-27L (~45 g, 25 m, 21.6 kHz, ~$142)**
for 2D SLAM now, with a small depth camera later for below-plane obstacles. Everything in the sim
plan (§5) is identical regardless of which unit you buy — start it today.

## 1. A2M12 assessment (datasheet-verified)
| Item | Value |
|---|---|
| Range | 0.2–12 m (white), 10 m black |
| Sample/scan rate | 16 kHz; 10 Hz typ (5–15), 0.225° @10 Hz (1600 pts/rev) |
| Tech | Triangulation (indoor; ~1% ≤3 m, ~2.5% ≥5 m) |
| Size / weight | 76 mm dia × 41 mm; **190 g bare, ~230–250 g installed** |
| Power | 5 V ±, ripple ≤50 mV: 450–600 mA run, **1.5 A spin-up, 2.5 A inrush** |
| Interface | 3.3 V TTL UART **256000 baud** via CP2102 USB adapter (kit) |
| Price | ~$229 (DFRobot, out of stock at check), €197–219 Husarion; spotty availability |

On BARQ: ~9% of robot mass on the top deck; CoM rises ~6 mm; pitch/roll inertia +~7%; the 76 mm
puck dominates the 117 mm-wide deck. Not disqualifying — but measurable. Whatever is mounted,
its mass/inertia goes into the URDF (RL trains on the robot that exists).

## 2. Wiring & driver reality (Jetson Orin Nano + ROS 2 Humble)
- Driver: `ros-humble-rplidar-ros` 2.1.4 **in the Humble apt repo** → `rplidar_a2m12_launch.py`
  (port /dev/ttyUSB0, 256000, frame `laser`, topic `/scan`). Alternative `sllidar_ros2` is
  source-build; its a2m12 launch filename on main literally contains a space — use the `view_` one.
- Adapter enumerates via CP2102 (10c4:ea60); `cp210x.ko` **present** on this Jetson (verified).
- **LANDMINE (verified live): `brltty` 6.4-4ubuntu3 is installed on the BARQ Jetson host** — this
  exact version hijacks CP2102 and makes /dev/ttyUSB0 vanish. `sudo apt remove brltty` BEFORE the
  lidar arrives. Also: user not in `dialout` → `sudo usermod -aG dialout barq` (or udev rule on
  the HOST, then `--device` into Docker).
- Power: never from a Jetson USB port long-term (inrush profile = brownout/re-enumeration risk).
  Dedicated ≥2 A 5 V BEC, shared GND, LC filter; the CP2102 carries only TX/RX.

## 3. Making 2D useful on a WALKING robot
Scan plane ≈ 0.22 m up, body pitched 4.5° nose-down + few degrees of trot oscillation:
- Static 4.5°: forward beam strikes the floor at **~2.8 m** — a permanent floor-return arc; no
  12 m forward view. At 8 m the plane is 0.63 m below horizontal (and above, behind).
- With ±2.5° oscillation the floor-strike arc **sweeps 1.8–6.3 m every gait cycle** — a moving
  phantom wall for scan matchers and costmaps. (Range bias to real walls is minor: ~0.75%.)

Mitigations in order of value: (1) **counter-wedge the mount +4.5°** (kills most of it);
(2) rigid mount + TF chain carrying IMU roll/pitch; (3) `ros-humble-laser-filters` box/shadow
filters in a gravity-aligned frame + **IMU-pitch-gated scan dropping** (trivial custom node —
prototype in sim); (4) slam_toolbox: non-zero minimum_travel_distance/heading so it integrates
across gait cycles; (5) nav2 obstacle layer with raytrace clearing / temporal decay.

**What 2D fundamentally cannot see for a 15-cm-tall robot:** everything below the 0.22 m plane —
shoes, cables, thresholds, toys, drop-offs/stairs, low overhangs. Standard complement: small nose
depth camera later (OAK-D Lite ~61 g / RealSense D435i ~72 g) + optional downward ToF cliff
sensors. Reserve the nose real estate now.

## 4. Alternatives
| Sensor | Weight | Range | Rate | Price | ROS2 Humble | Verdict (2.45 kg quadruped) |
|---|---|---|---|---|---|---|
| RPLidar A2M12 | 190 g | 12 m | 16 kHz | ~$229 | apt, excellent | Best driver story, worst mass/size ratio. Workable, not optimal. |
| RPLidar C1 | 110 g | 12 m | 5 kHz/0.72° | **$69** | apt (same pkg) | Budget DTOF; coarse for sparse rooms. |
| RPLidar A1M8 | 170 g | 12 m | 8 kHz/5.5 Hz | ~$99 | apt | Obsolete; skip (C1 replaces it). |
| LDROBOT LD19/D500 | **47 g** | 12 m | 4.5 kHz | ~$80–110 | official + maintained 3rd-party | Outstanding mass fraction (~2%). Solid. |
| **LDROBOT STL-27L** | **~45 g** | **25 m** | **21.6 kHz/0.167°** | **$142** | official (source build) | **Best technical fit** — beats A2M12 on every measurement spec at ~1/4 the weight (DTOF, 60 klux). |
| Unitree 4D L1 (3D) | 230 g | 30 m | 21.6k pts/s 3D + IMU | $249 | SDK node, source | Fixes the tilt problem but re-imports the mass problem + 12 V rail + 3D SLAM stack. Reconsider if outdoor/stairs become requirements. |

**Recommendation: STL-27L (first choice) or LD19/D500 (budget), not the A2M12.** The dominant
currency on a 2.45 kg robot is grams-at-height; the STL-27L is strictly better at 1/4 the weight
and 60% of the price; the A2M12's only edge is an apt-installable driver (cost of forgoing: one
colcon build). Budget a nose depth camera later — that covers BARQ's real hazard zone.

## 5. Sim-first integration plan (lidar-agnostic; do before buying)
Verified against the actual BARQ workspace (world has NO Sensors system yet; no IMU/lidar links;
/clock already bridged).

1. **World**: add the Sensors system to `barq_world.sdf`:
   `<plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">`
   `  <render_engine>ogre2</render_engine></plugin>`
   Headless caveat (Tegra): server-side rendering needs GL — either `--headless-rendering` in the
   server gz_args (EGL, **ogre2-only**) or fall back to `<render_engine>ogre</render_engine>`
   with the VNC X display (gz-sensors#504 documents ogre fallback + ogre2 grazing-angle lidar
   inaccuracies on ground planes — don't tune ground filters against sim returns). TEST FIRST:
   `ign topic -e -t /scan -n 1`.
2. **URDF**: `laser` link (mass 0.19 A2M12 / 0.047 STL-27L, cylinder 38×41) + fixed joint at
   `xyz="-0.04 0 0.075"` (aft-of-center deck; nose reserved for depth cam) with
   `rpy="0 -0.0785 0"` (**−4.5° counter-wedge**; verify sign in RViz), plus a `gpu_lidar` sensor:
   360°, samples 1600 (A2M12) / 2160 (STL-27L), range 0.2–12 / 25 m, noise σ=0.02,
   `<ignition_frame_id>laser</ignition_frame_id>` (else the frame is the scoped sensor name),
   topic `scan`, 10 Hz. Frame/topic chosen for zero-config parity with the real driver.
3. **Bridge**: `'/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'`.
4. **Odometry stopgap** (slam_toolbox REQUIRES odom→base_link TF; BARQ publishes none):
   Fortress `OdometryPublisher` system (odom_frame `odom`, robot_base_frame `base_link`,
   50 Hz, `<dimensions>3</dimensions>` to keep roll/pitch) + bridge
   `/model/barq/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry` and
   `/model/barq/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V` remapped to `/tf`.
   Loudly ground-truth-only: makes SLAM look perfect; swapping in real legged odometry later is
   exactly the sim-vs-real delta to study.
5. **slam_toolbox** (apt: `ros-humble-slam-toolbox`): `online_async_launch.py use_sim_time:=true`
   with `odom_frame: odom`, `base_frame: base_link`, `scan_topic: /scan`, `mode: mapping`,
   `minimum_travel_distance: 0.2`, `minimum_travel_heading: 0.3`.
   Acceptance experiment: drive around, watch `/map` build; then zero the wedge (rpy 0) and watch
   the floor-strike arc appear at ~2.8 m — the cheapest rehearsal of the hardware failure mode.

**Unverified/lower confidence:** Orin Nano per-port USB current limit; LD19 kit exact price;
STL-27L 45 g (vendor pages, not triple-sourced); Unitree L2 weight; A1M8 weight/price.
