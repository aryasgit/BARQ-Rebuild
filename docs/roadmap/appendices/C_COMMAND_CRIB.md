# Appendix C — Command Crib

> Appendix · verified against repo @ 0e5ddaf

Every command that matters, copy-paste ready. Convention: **host** = Jetson shell (no ROS on the
host); **container** = inside `barq:dev`; **Mac** = the operator's machine. Anything `ros2 ...`
non-interactive gets `timeout -k 2 N` (the CLI swallows SIGTERM under load — failure tree A2).

## HOST (Jetson)

| Command | When / why |
|---|---|
| `cd ~/barq_ws/src && git status -sb` | sanity before/after work; expect `## stage-2...origin/stage-2` |
| `git push` | after each milestone commit (D-013). GitHub :22 is blocked here — `~/.ssh/config` already routes `github.com` → `ssh.github.com:443`; author = Aryaman Gupta |
| `export PATH="$HOME/.local/bin:$PATH"` | PlatformIO lives in `~/.local/bin`; needed once per shell |
| `cd ~/barq_ws/src/barq_firmware && pio test -e native` | firmware codec unit tests on the host — expect `6 Succeeded` (needs `test_build_src = yes`, already in platformio.ini) |
| `pio run -e teensy41` | full firmware build → `.pio/build/teensy41/firmware.hex`; expect `SUCCESS` |
| `pio run -e teensy41 -t upload` | flash (Teensy on USB; press its button if first flash) |
| `sudo ~/fix_display.sh` | headless `:0` black/640×480 → forces DP-0 connected @1024×768, restarts gdm. Survives reboots (xorg.conf persists) |
| `~/setup_vnc.sh` | start x11vnc on :5900. **Re-run after every Jetson reboot** (/tmp wiped) |
| Mac Finder → `vnc://barq.local:5900` | view the Jetson desktop (fallback `vnc://<ip>:5900`) |
| `DISPLAY=:0 ~/run_barq_gui.sh` | interactive container with X passthrough for RViz/GUIs on :0 |
| `tegrastats` | live CPU/GPU/mem/temps — first stop for "is the Orin saturated?" (Ctrl-C to quit) |
| (Mac) `barq-send <files>` / `barq-get <remote> ` | rsync channel Mac↔Jetson; run in a LOCAL Mac terminal, never inside ssh. `-t <subdir>` targets `~/<subdir>`; flaky mDNS: `export BARQ_HOST=barq@<ip>` |

## DOCKER

```bash
# Canonical interactive dev container (THE run line — /dev/shm mount is non-negotiable, A1):
docker run --runtime nvidia -it --rm --network host --shm-size=8g \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev

# Detached sim (background service pattern; name it so exec/logs/stop work):
docker run --runtime nvidia -d --name barq_sim --network host --shm-size=8g \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev bash -lc '
    source /opt/ros/humble/setup.bash && source /root/barq_ws/install/setup.bash &&
    ros2 launch barq_bringup sim.launch.py gait:=true gui:=false'
sleep 45 && docker logs barq_sim --tail 20    # verdict from the LOG TAIL, never the exit code (A4)

# REAL ROBOT container (P3+): serial passthrough + RT scheduling caps (Q-009):
docker run --runtime nvidia -it --rm --network host --shm-size=8g \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws \
  --device /dev/ttyACM0 --device /dev/ttyUSB0 \
  --cap-add=SYS_NICE --ulimit rtprio=99 \
  barq:dev
# (after the P1 udev rules: prefer --device /dev/teensy --device /dev/lidar — A19)

# Exec pattern (ALWAYS source both, ALWAYS timeout -k 2 on ros2):
docker exec barq_sim bash -lc '
  source /opt/ros/humble/setup.bash && source /root/barq_ws/install/setup.bash &&
  timeout -k 2 10 ros2 topic list'

# One DDS stack at a time (host network = ONE shared graph, A21). Must run two? Isolate:
export ROS_DOMAIN_ID=42        # same value in EVERY shell of that stack
# (integration_pty.py self-isolates on 42 already)

# Build the workspace (inside any container):
cd /root/barq_ws && colcon build --symlink-install 2>&1 | tail -5   # check the tail!
# Fresh clone ONE-TIME extra (or sim is silently soft, A10):
GZ_VERSION=fortress colcon build --packages-select gz_ros2_control
```

## SIM

```bash
# Full launch arg matrix (all optional; defaults shown):
ros2 launch barq_bringup sim.launch.py \
  world_file:=barq_world.sdf \   # or barq_course.sdf (10x8 m doorway+slalom+boxes)
  gui:=false \                   # true = Gazebo GUI (forced ogre on Tegra; NEVER during missions)
  gait:=false \                  # true = ik_node + gait_planner (walk via /cmd_vel)
  slam:=false \                  # true = slam_toolbox mapping from the sim lidar
  nav:=false \                   # true = nav2 (use with slam:=true)
  odom_source:=ground_truth \    # estimated = legged odometry owns odom->base_link TF
  foot_mu:=0.9 \                 # foot-ground friction (D-018 sweep knob)
  gait_duty:=0.6 \               # 0.6 fast/veers-left, 0.55 dead straight (Q-016)
  gait_period:=0.5               # gait cycle seconds

# Drive / stop (drive loop must outlive the 1 s cmd_vel deadman):
timeout -k 2 12 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12}}"
timeout -k 2 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# Replay demo from the host (teleports to origin, walks DUR seconds at VX):
~/walk_demo.sh            # 60 s @ 0.12
~/walk_demo.sh 120 0.18

# Teleport (container; robot STOPPED first — mid-stride teleport corrupts the next run, A22):
ign service -s /world/barq_world/set_pose --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean --timeout 3000 \
  --req 'name: "barq", position: {x: 0.0, y: 0.0, z: 0.148}'

# Ground-truth pose without any GUI (the headless oracle):
ign model -m barq --pose

# Walk regression metric (gait running; prints one parseable WALK line):
python3 /root/barq_ws/src/diagnostics/sim_walk_metric.py --vx 0.15 --duration 10
# expect: realized ≥55 % @ duty 0.6, fresh spawn (D-019)

# Actuation probes (D-018 fidelity tools; same metrics st3215_diag measures on the bench):
python3 /root/barq_ws/src/diagnostics/sim_actuation_probe.py step --joint FL_knee_joint --delta 0.3
# expect: rise ~50 ms, peak vel ≤ 4.71 rad/s, no overshoot (k=60/s). ~200 ms rise = soft plugin (A10)
python3 /root/barq_ws/src/diagnostics/sim_actuation_probe.py track --duration 12

# Tracking metric OF RECORD (immune to rclpy starvation, A11): record then analyze offline
ros2 bag record -o /tmp/track /joint_states /joint_group_position_controller/commands
python3 /root/barq_ws/src/diagnostics/analyze_track_bag.py /tmp/track/track_0.db3
# expect: mean RMS ~18 mrad @ k=60/s (75–93 mrad = soft plugin)
```

## REAL (Stage-4 stack; PTY today, Teensy on hardware day)

```bash
# Emulator (REAL firmware LoopCore on a PTY; prints "PTY /dev/pts/N"):
ros2 run barq_hw teensy_emulator

# The robot launch line — IDENTICAL for emulator and hardware (the drop-in contract):
ros2 launch barq_bringup real.launch.py device:=/dev/pts/N    gait:=true   # emulator
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true   # flashed Teensy
# (post-P1 udev: device:=/dev/teensy)

# Integration test, 9 checks, zero hardware (spawns its own emulator, DDS domain 42):
ros2 run barq_hw integration_pty.py        # expect 9/9, exit 0
# Hardware-day equivalent against the REAL Teensy: P3's procedure in
# phase-3-firmware-integration/ (same checks, /dev/ttyACM0) — phase doc is authoritative.
```

## DIAGNOSTICS — `st3215_diag.py` (bench servo tool; pyserial only, no ROS/Teensy)

Global flags: `--port <dev>` (default auto-detect), `--baud 1000000`.

| Subcommand | Purpose |
|---|---|
| `./st3215_diag.py plan` | print the BARQ ID map 0–11 (one driver board per leg) — matches `robot_params.yaml servos:` |
| `./st3215_diag.py scan [--max-id 30]` | find every servo on the bus; first check for any bus problem (A13/A14) |
| `./st3215_diag.py ping <id>` | single-servo liveness |
| `./st3215_diag.py status <id>` | volts / temp / mode / errors — health snapshot |
| `./st3215_diag.py monitor <id> [--hz 5]` | live position/load/temp stream; use while hand-moving a torque-off joint |
| `./st3215_diag.py torque <id> on\|off` | torque enable; **off before handling** |
| `./st3215_diag.py set-id <old> <new>` | assign BARQ ID — with exactly ONE servo connected; label it |
| `./st3215_diag.py calibrate-mid <id>` | hand-hold mechanical middle → becomes 2048 (Feetech torque=128 trick); BEFORE assembly |
| `./st3215_diag.py move <id> [pos] [--deg D] [--speed 800] [--acc 50]` | commanded move; watch it arrive |
| `./st3215_diag.py sweep <id> [--amp-deg 30 --cycles 3 --freq 0.25]` | sine sweep with tracking-error report — the bench twin of `sim_actuation_probe` (P3 bench ID) |
| `./st3215_diag.py limits <id> [--min N --max N]` | read/write servo-side angle limits |

## SLAM / NAV

```bash
# Mission pattern (sim or robot): NO Gazebo GUI on the Jetson; goals via a ROBUST python
# action client (explicit accept + result future) — the `ros2 action send_goal` CLI loses
# goals silently under load (A6). Canonical client + mission checklist:
# phase-5-perception-autonomy/. Manual path: RViz "2D Goal Pose" button.

# Save the map (PGM+YAML, for reuse / records):
timeout -k 2 20 ros2 run nav2_map_server map_saver_cli -f /root/barq_ws/maps/room1
# Serialize the SLAM pose-graph (re-localizable, slam_toolbox native):
timeout -k 2 20 ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: /root/barq_ws/maps/room1_graph}"

# Speed keys (barq_nav2.yaml; the only two places speed lives — change BOTH):
#   FollowPath.desired_linear_vel: 0.22      (sim; hardware starts 0.10 per phase-5)
#   velocity_smoother.max_velocity: [0.22, 0.06, 0.5]
# Live retune mid-mission (persists only until restart — then edit the yaml):
timeout -k 2 10 ros2 param set /controller_server FollowPath.desired_linear_vel 0.15
```

## DEBUG

```bash
timeout -k 2 7 ros2 topic hz /scan            # lidar alive? sim: ~9.7 Hz light / ~7 Hz loaded
timeout -k 2 7 ros2 topic hz /joint_states    # control loop health: ~100 Hz, low jitter
timeout -k 2 5 ros2 topic echo --once /odom_est   # estimator output sanity
timeout -k 2 5 ros2 topic echo --once /imu/data   # IMU flowing (estimator silently waits on it)

# TF chain checks (the SLAM-eating failure is always here first, A5):
timeout -k 2 7 ros2 run tf2_ros tf2_echo odom base_link
timeout -k 2 7 ros2 run tf2_ros tf2_echo odom laser   # expect z≈0.220, rotation ≈0 at stance

# Plugin-gain canary (sim up; THE check that the patched overlay is live, A10):
timeout -k 2 10 ros2 param get /gz_ros2_control position_proportional_gain
# 0.6 = patched overlay · 0.1 = soft /opt fallback → rebuild gz_ros2_control

# Controller state:
timeout -k 2 10 ros2 control list_controllers   # expect both active

# Load / thermals (host): tegrastats
# Container logs (the truth source): docker logs barq_sim --tail 30
```
