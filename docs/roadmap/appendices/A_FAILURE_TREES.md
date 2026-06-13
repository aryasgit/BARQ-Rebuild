# Appendix A — Failure Trees

> Appendix · verified against repo @ 0e5ddaf

Symptom-indexed. Find your symptom, walk the CHECK ladder top-down (cheapest first), apply the
FIX for the first check that bites. Entries marked **[hit]** have already happened to this
project (source: `docs/03_CHANGELOG.md`, `docs/05_RESEARCH_LOG.md`); the rest are predictable
hardware failures with pre-planned ladders. Meta-rule when nothing matches:
`00_DOOMSDAY_PROTOCOL.md` §4 (reproduce twice → bisect at the seams → swap-test).

**Index**
| # | Symptom | Domain |
|---|---|---|
| A1 | RViz empty across containers | DDS |
| A2 | `ros2` CLI hangs in `docker exec` | tooling |
| A3 | Launch aborts silently, no nodes | xacro |
| A4 | Build "succeeded" but artifacts wrong/missing | build |
| A5 | slam_toolbox drops every scan ("queue full") | TF |
| A6 | nav2 goal accepted-then-nothing / never accepted | compute |
| A7 | Odometry/estimator flatlined at ~0 | estimator |
| A8 | Walks but veers left/right | gait |
| A9 | Realized speed ~half of commanded | gait (expected) |
| A10 | Sim servos feel soft / tracking lags | sim plugin |
| A11 | Live metric numbers look impossible | measurement |
| A12 | Serial device missing or renamed | udev/host |
| A13 | One servo not responding on a bus | servo bus |
| A14 | Entire servo bus dead | servo bus |
| A15 | Jetson reboots / browns out under load | power |
| A16 | Servo hot (>60 °C) | thermal |
| A17 | IMU yaw drifts / orientation wrong | IMU |
| A18 | Robot tips backward at stance/stand-up | balance |
| A19 | Teensy and lidar swap device names across boots | udev |
| A20 | Gazebo GUI black / meshes invisible | sim GUI |
| A21 | Two ROS stacks cross-polluting each other | DDS |
| A22 | Run-to-run results inconsistent (sim or hw) | method |
| A23 | Controllers refuse to activate on `mode:=real` | hw interface |
| A24 | Robot collapses/snaps at spawn or power-on | startup pose |
| A25 | SLAM maps nothing in an open area | perception |

---

### A1 — RViz (or any viewer) sees topics but shows NOTHING, across containers **[hit]**
- SYMPTOM: viewer container lists topics, discovery fine, but no data arrives — empty RViz, no TF, no /clock.
- CHECK IN ORDER:
  1. Was the viewer container started with `-v /dev/shm:/dev/shm`? (`docker inspect <c> | grep shm`)
  2. `timeout -k 2 5 ros2 topic hz /tf` inside the viewer container — 0 msgs confirms data-plane loss.
  3. RViz model missing only when started late → robot_description QoS (see fix 2).
- ROOT CAUSES SEEN: (a) FastDDS same-host transport is **shared memory**; private container /dev/shm ⇒ UDP discovery works, data silently never arrives. (b) `barq_slam.rviz` robot_description sub was Volatile → late joiners never got the model.
- FIX: (a) run **every** ROS container with `-v /dev/shm:/dev/shm` (canonical run lines: `C_COMMAND_CRIB.md`). (b) Transient Local durability on robot_description displays (already in repo rviz configs).
- PREVENTION: never hand-write a `docker run` for ROS — copy from the crib. Data ≠ discovery: always verify with `topic hz`, not `topic list`.

### A2 — `ros2` CLI hangs forever inside `docker exec` **[hit — wedged four sessions]**
- SYMPTOM: `ros2 topic echo/hz/pub` in an exec never returns; plain `timeout N` doesn't kill it either.
- CHECK: are you using `timeout -k 2 N ros2 ...`? That IS the ladder.
- ROOT CAUSE SEEN: the ros2 CLI swallows SIGTERM under load; `timeout` without `-k` never escalates to SIGKILL.
- FIX: **always** `timeout -k 2 <N> ros2 ...` in non-interactive shells. Wedged already? `docker exec <c> pkill -9 -f "ros2 topic"`.
- PREVENTION: standing rule in HANDOFF; the crib writes every debug command with `timeout -k 2`.

### A3 — Launch dies before ANY node starts; xacro "worked" by hand **[hit]**
- SYMPTOM: `ros2 launch` exits early/silently; `xacro file.xacro` at a shell looks fine.
- CHECK IN ORDER:
  1. `xacro <file> mode:=gazebo 2>err.txt >/dev/null; wc -c err.txt` — **must be 0 bytes**.
  2. Read err.txt: warnings count as failure (the launch `Command` substitution treats ANY stderr as fatal).
- ROOT CAUSE SEEN: macro params named `min`/`max` shadowed xacro builtins → "redefining global symbol" **warnings on stderr** → launch aborted with no node ever starting.
- FIX: rename offending macro params (repo precedent: `min/max` → `lower/upper`); re-check stderr is empty.
- PREVENTION: gate any URDF/xacro edit on `xacro … 2>&1 >/dev/null | wc -c` == 0 plus `check_urdf`.

### A4 — Build reports success but the thing you built is wrong/missing **[hit — sim image build #4]**
- SYMPTOM: docker/colcon exit code 0 (or the task wrapper says "done") but later behavior is stale/broken.
- CHECK IN ORDER:
  1. Read the actual **log tail**: `docker logs <c> --tail 30` / last lines of the colcon log — exit codes lie under load.
  2. apt layer: grep the build log for `dpkg: error` / `trying to overwrite`.
  3. colcon: `colcon build ... 2>&1 | tail -20` and check `N packages finished` matches expectation.
  4. Verify the artifact itself (file timestamp, `dpkg -l | grep <pkg>`, run the binary).
- ROOT CAUSES SEEN: (a) dustynv CUDA-OpenCV base image conflicts with Ubuntu `libopencv-dev` pulled by the `ros-gz` metapackage and by nav2's dep tree — build dies in the apt layer while the wrapper reads success.
- FIX: slim explicit package set (`ros-gz-sim ros-gz-bridge ign-ros2-control`, NOT the metapackage); for nav2: `--no-install-recommends` + `-o Dpkg::Options::=--force-overwrite` (headers only — repo Dockerfile shows the exact lines).
- PREVENTION: every build verdict comes from the log tail + a smoke test of the artifact, never the exit code (Protocol §2).

### A5 — slam_toolbox registers the sensor then drops EVERY scan: "queue is full" **[hit]**
- SYMPTOM: /scan flows, slam_toolbox starts, zero map updates, log spams queue-full.
- CHECK IN ORDER:
  1. `timeout -k 2 5 ros2 run tf2_ros tf2_echo odom laser` — does the chain resolve with the EXACT names `odom → base_link → laser`?
  2. `timeout -k 2 5 ros2 topic echo --once /tf | grep frame_id` — look for prefixed frames (`barq/odom`, `barq/base_link`).
  3. Check who publishes odom→base_link: exactly ONE owner (odom_tf node, estimator, or — later — the hw estimator).
- ROOT CAUSE SEEN: gz plugins model-prefix their TF frames (`barq/odom`) → slam_toolbox's `odom_frame: odom` never matches → silently drops all scans.
- FIX: the repo's `odom_tf` node re-publishes /odom as TF with **forced** frame names (sim.launch.py wires it). On hardware the estimator owns the TF with the same names.
- PREVENTION: any new TF publisher must use the bare names in `barq_slam.yaml` (`odom`/`map`/`base_link`/`laser`); verify with tf2_echo before debugging SLAM internals.

### A6 — nav2 goal silently lost (sent, accepted or not, robot static) **[hit]**
- SYMPTOM: goal sent, no error, no motion; or CLI action call times out at the handshake.
- CHECK IN ORDER:
  1. Load: `tegrastats` (host) — CPU pegged? GUI renderers running? (`docker top`/`ps` for `ign gazebo -g`, rviz2)
  2. `timeout -k 2 5 ros2 topic hz /plan` and `/cmd_vel` — did planning happen but the controller starve, or nothing at all?
  3. bt_navigator / controller_server logs (`docker logs`) — look for handshake/timeout lines, not exit status.
- ROOT CAUSE SEEN: full sim load (physics + lidar rendering + SLAM + nav2 + 2 GUI renderers) on one Orin → action handshake timed out → goal silently lost.
- FIX: shed GUIs during missions (no Gazebo GUI on the Jetson; RViz is the mission view, ideally on a viewer machine); send goals with a **robust python action client** (explicit accept + result future, repo precedent), never the `ros2 action` CLI.
- PREVENTION: mission protocol = no robot-side GUIs, verify by telemetry not exit codes. Note: the two biggest sim loads (physics, rendering) do NOT exist on the real robot; if this hits on hardware, suspect SLAM+nav2+estimator budget → see the perception phase folder (compute budget).

### A7 — Odometry/estimator output flatlined near zero while the robot clearly moves **[hit]**
- SYMPTOM: /odom_est ≈ 0.001 m while ground truth (or tape measure) says ~1 m.
- CHECK IN ORDER:
  1. Inputs alive? `timeout -k 2 5 ros2 topic hz /imu/data /joint_states` (estimator silently waits for IMU).
  2. Log which legs the stance heuristic picks over one gait cycle — is it ALWAYS the same pair?
  3. Re-derive: does any stance/posture trim (rear_raise!) bias the heuristic?
- ROOT CAUSE SEEN: "two lowest feet = stance" + D-016 rear_raise ⇒ both REAR legs always selected; per-leg cyclic motion averages zero ⇒ odometry flatlined (0.0015 m vs 0.86 m truth).
- FIX: compare trot **diagonal z-sums** (each diagonal = one front + one rear; the trim cancels). In repo: `state_estimator_node.stance_legs()`, locked by unit test.
- PREVENTION: audit estimator assumptions against every gait/stance FEATURE (trims, asymmetries), not just the nominal gait. New trim ⇒ rerun the estimator drift check (walking phase folder).

### A8 — Robot walks but veers left/right (open loop)
- SYMPTOM: straight-line cmd produces a curved path.
- CHECK IN ORDER:
  1. Sim or hw? In sim with fresh spawn: known duty-dependent bias (Q-016) — duty 0.50 → right veer, 0.60 → left veer, **0.55 ≈ straight**. Not a bug; a mapped trade.
  2. (hw) Per-servo `zero_offset` asymmetry: re-check assembled-neutral vs 2048 on each servo (`st3215_diag.py status/monitor`, calibration phase folder procedure).
  3. (hw) Mechanical: bracket/horn slip on one leg (witness-mark check), foot wear asymmetry.
  4. Measure, don't eyeball: 10 s walk, fresh start, log yaw drift (sim: `sim_walk_metric.py`; hw: estimator yaw or tape + protractor).
- ROOT CAUSES SEEN: duty-cycle yaw bias zero-crossing ~0.55 (mechanism open, Q-016); hardware zero-offset asymmetry is the predicted twin.
- FIX: open-loop demos `gait_duty:=0.55`; missions keep duty 0.6 (nav2 RPP closes heading — obstacle course completed with worse). Hardware: fix calibration first, only then touch gait.
- PREVENTION: candidate proper fix on record: estimator yaw-rate feedback into `wz` (~20 lines) — measure before/after for the log (Q-016).

### A9 — Realized speed is ~half the commanded speed — **EXPECTED, not a bug** **[hit, SOLVED Q-013/D-019]**
- SYMPTOM: cmd 0.15 m/s, realized ~45–60 %.
- CHECK (only if it gets WORSE than ~55 % at duty 0.6, fresh spawn):
  1. `sim_walk_metric.py --vx 0.15 --duration 10` from a fresh spawn — compare to the D-019 map (duty 0.50 → 51 %, 0.55 → 47 %, 0.60 → 57–62 %).
  2. Plugin-gain canary (A10) — soft servos also cost speed.
  3. Tracking RMS via bag (A11 method): should be ~18 mrad; 75+ mrad means actuation, not gait.
- ROOT CAUSE (proven by μ-invariance, μ 0.25–0.9 sweep): swing-foot DRAG — grounded swing feet slide forward against stance push; both forces Coulomb so μ cancels. Front clearance is reach-capped ~20 mm (2 mm from the tibia −2.2 fold limit); trot heave eats it.
- FIX (already in): smoothstep swing + duty 0.6 → ~60 % realized. Full fix needs body-state feedback in the gait or RL (RL phase folder).
- PREVENTION: don't "tune friction" to chase this — the null result already eliminated that whole hypothesis class. Budget missions at realized ≈ 0.6 × commanded.

### A10 — Sim falls back to the SOFT servo plugin (k=10/s) **[hit — the construction-order bug]**
- SYMPTOM: sluggish joints, ~200 ms step rise, trot tracking RMS 75–93 mrad, speed down; everything else "normal".
- CHECK IN ORDER:
  1. Canary: `timeout -k 2 10 ros2 param get /gz_ros2_control position_proportional_gain` (sim up). **0.6 = good; 0.1 = soft fallback.**
  2. Is the overlay built? `ls ~/barq_ws/install/gz_ros2_control` — absent ⇒ sim.launch.py silently shadows nothing.
  3. Step metric: `sim_actuation_probe.py step` — rise should be ~50 ms (k=60/s), not ~200 ms.
- ROOT CAUSE SEEN: upstream ign_ros2_control 0.7.x constructs its node BEFORE reading `<parameters>` → the gain is unreachable by config; repo carries a vendored 3-line patch (`external/gz_ros2_control`, BARQ.patch, PROVENANCE.md). Fresh clones that skip the overlay build fall back to the /opt binary.
- FIX: `GZ_VERSION=fortress colcon build --packages-select gz_ros2_control` once per workspace, rebuild, relaunch; re-check the canary.
- PREVENTION: the canary check is part of the P0 environment gate and the regression spine (`B_ACCEPTANCE_GATES.md`).

### A11 — Live-sampled metrics are garbage under load (impossible rates, fake gaps) **[hit twice]**
- SYMPTOM: rclpy-based probe reports numbers that contradict physics/telemetry; loaded Jetson.
- CHECK IN ORDER:
  1. Sample-rate sanity: does the probe's own receive rate match the topic's nominal rate? If not, the PROBE is starving — discard the data.
  2. Re-measure offline: `ros2 bag record` (C++ recorder, starvation-immune) → analyze the bag.
- ROOT CAUSE SEEN: rclpy executor starvation on a loaded Jetson produced plausible-looking garbage twice.
- FIX: bag-based pipeline: `ros2 bag record -o /tmp/track /joint_states /joint_group_position_controller/commands` then `python3 diagnostics/analyze_track_bag.py /tmp/track/track_0.db3`.
- PREVENTION: every metric gates on its own sample-rate sanity check; anything measured live on a loaded box is provisional until a bag confirms it.

### A12 — Serial device missing, busy, or renamed (`/dev/ttyACM0`/`ttyUSB0` not there)
- SYMPTOM: driver board / Teensy / lidar absent or permission-denied.
- CHECK IN ORDER:
  1. `ls -l /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/ 2>/dev/null` — enumerated at all?
  2. `dmesg | tail -20` right after replug — does the kernel see it? Disconnect loop = cable/power.
  3. Permission denied → `groups` must include `dialout` (`sudo usermod -aG dialout $USER`, re-login).
  4. Device appears then vanishes / is grabbed → **`brltty` hijacks CP2102 UARTs**: `sudo apt remove brltty` (found live on this Jetson, Q-015 prep).
  5. In-container: was the device passed through (`--device /dev/ttyACM0` or `-v /dev:/dev`)?
- ROOT CAUSES SEEN/EXPECTED: brltty (confirmed present on stock Ubuntu), missing dialout, container without passthrough, USB cable that is charge-only.
- FIX: as per check; then address by stable path `/dev/serial/by-id/...`, never by ACM index.
- PREVENTION: udev rules with SYMLINK (A19) installed during the electronics phase; crib carries the by-id pattern.

### A13 — One servo not responding on a bus (others fine)
- SYMPTOM: `st3215_diag.py scan` misses one ID; or a joint is limp/ignored under the Teensy.
- CHECK IN ORDER:
  1. Bench-isolate it: ONE servo on the bench board → `st3215_diag.py scan` (factory ID is 1; a "missing" servo often answers at a clashing/old ID).
  2. ID clash: two servos on the same bus with one ID → scan shows one, behavior erratic. Re-ID with only one connected (`set-id`).
  3. Wiring: 3-pin daisy order/seating; swap the servo onto a known-good pigtail.
  4. Baud ladder: try `--baud 1000000`, then 500000, 250000, 128000, 115200 — a servo accidentally reconfigured answers on the wrong rate; set it back at that rate.
  5. `status <ID>`: voltage in range? torque flag? error bits? Over-temp/over-load protection latches until power cycle.
  6. Still dead on a known-good board+cable+12 V → servo is DOA/dead: swap in a spare, RMA.
- ROOT CAUSES EXPECTED: ID clash (assigning with >1 connected), wrong baud after a botched write, seated-but-not-latched connector, dead servo (batch variance — risk D-R1).
- FIX: per rung; record the servo's serial + fate in the bench log (calibration phase folder).
- PREVENTION: assign IDs with exactly one servo connected; label physically; bench-certify every servo BEFORE assembly (diagnostics/README.md procedure).

### A14 — An ENTIRE bus dead (all 3 servos of a leg silent)
- SYMPTOM: scan finds nothing on one driver board / one Teensy UART; other legs fine.
- CHECK IN ORDER (swap-matrix — change ONE element per test):
  1. Power: 12 V present at the driver board terminals (DMM)? Fuse/connector upstream?
  2. Move the same servo chain to a KNOWN-GOOD board → works ⇒ board or its upstream is the fault.
  3. Move the suspect BOARD to the known-good position (cable/port) → still dead ⇒ board dead.
  4. Replace the signal cable (USB on bench / UART harness on robot) — cables fail more than boards.
  5. Bench board via USB but robot bus via Teensy? Then suspect the Teensy UART pin/wiring next (firmware phase folder pin map), test with loopback.
- ROOT CAUSES EXPECTED: unpowered board (screw terminal loose — see A15 wiring discipline), dead board, broken harness, wrong UART pin.
- FIX: per matrix outcome; spares exist for board + cable (BOM, environment phase folder).
- PREVENTION: strain-relief and witness-mark every screw terminal at assembly; one driver board per leg is the deliberate isolation seam — use it.

### A15 — Brownout symptoms: Jetson reboots / servos all twitch-release under stance load
- SYMPTOM: power-cycles or whole-rail sag when the robot loads its legs (stand-up is the worst case).
- CHECK IN ORDER:
  1. Battery first: pack voltage at rest ≥ 13.6 V? (Below the floor everything is undefined — charge first.)
  2. Telemetry replay: STATE `vbus` field around the event (bag/log) — rail dip below the buck's regulation floor?
  3. Buck output under static load: DMM on the 12 V rail while pressing the body down (cradle) — sag below ~11.7 V = undersized/mis-trimmed buck (power phase folder worksheet TBDs).
  4. Connectors: thermal-image or touch-test XT60s/screw terminals after a stance hold — a warm joint is a resistive joint.
  5. Separate the rails: does the Jetson (direct 4S, fused) also reset, or only the servo rail? Jetson reset with healthy 4S ⇒ its own feed/fuse, not the buck.
- ROOT CAUSES EXPECTED: buck continuous rating below stance-transient draw (THE load-bearing new component — risk register D-R4), battery sag near floor, resistive joint.
- FIX: re-run the power phase folder sizing worksheet with the MEASURED stance currents (its TBD table); upsize buck / parallel second buck per its fallback ladder; replace the offending joint.
- PREVENTION: the supervised brownout rehearsal gate (power phase folder) before any untethered work; INA260 logging during all early stands; battery floor discipline 13.6 V.

### A16 — Servo hot: temp_max (STATE) or `st3215_diag monitor` above 60 °C
- SYMPTOM: hottest-servo telemetry climbing past 60 °C during stance/walk.
- CHECK IN ORDER (escalating):
  1. ≥ 60 °C: note WHICH servo; torque-off within 2 min (cradle or sit pose); let it cool; identify why THAT one (stance load asymmetry? binding linkage? fighting its neighbor?).
  2. Hand-rotate the joint torque-off: grinding/stiff spots = mechanical bind → fix the mechanics, not the duty cycle.
  3. Load telemetry (`load[12]` in STATE / `monitor`): sustained high static load in stance ⇒ posture problem — revisit stand_height/trim (deeper crouch = more torque).
  4. ≥ 65 °C ever, or repeated 60 °C at rest poses: stop; re-check supply voltage (over-voltage heats), consider torque-limit staging (firmware phase folder) and duty-cycling stands.
- ROOT CAUSES EXPECTED: static stance torque concentration (one servo carrying the trim), binding from assembly tolerance, ambient + enclosed mounting.
- FIX: mechanical first, posture second, gait duty last. The ST3215's own over-temp protection is the LAST line, not the plan.
- PREVENTION: 60/65 °C thresholds become firmware faults (parameter registry TBD rows, walking phase folder safety layer); log temp_max in every endurance run.

### A17 — IMU yaw drifts / orientation wrong / estimator heading rotates at rest
- SYMPTOM: stationary robot's yaw creeps; or estimated heading offset from reality.
- CHECK IN ORDER:
  1. (BNO085) Calibration status via SH-2 (firmware exposes it — firmware phase folder): mag/gyro cal < 2 ⇒ run the calibration dance (figure-8, rotations) away from steel.
  2. Mount transform: is the IMU's axes→base_link rotation in the firmware/URDF actually the PHYSICAL mounting orientation? 90°/180° errors look like "drift" once moving.
  3. Magnetic environment: yaw jumps near motors/power wiring ⇒ prefer the gyro-integrated/game-rotation output over magnetometer-fused yaw indoors (firmware phase folder decision).
  4. I2C health under motor load: SH-2 reset counts / stale flags (fault bit1) — see risk D-R8 (bus lockups under noise); shielded/short wiring, pull-up check.
  5. Quantify: 10 min static log of yaw — BNO085-class drift should be small; degrees/minute at rest = config/cal problem, not sensor death.
- ROOT CAUSES EXPECTED: uncalibrated magnetometer indoors, wrong mount transform, EMI on I2C.
- FIX: per rung; record the chosen rotation-vector variant + mount transform in the parameter registry when set (TBD row).
- PREVENTION: IMU bring-up gate on the bench BEFORE it's buried in the body (electronics phase folder).

### A18 — Robot tips/rocks BACKWARD at stance or during stand-up
- SYMPTOM: weight visibly on the rear legs; backward topple on stand-up or at gait start.
- CHECK IN ORDER:
  1. Is the D-016 trim active? `rear_raise` must be 0.02 in BOTH gait_planner and ik_node params (defaults are — check no override zeroed it).
  2. Battery/payload position changed since the trim was tuned? The 512 g pack dominates CoM — re-seat to the documented bay position first.
  3. Measure, then trim: raise `rear_raise` in +0.005 steps (cradle → spotter → free, per Protocol §3), watching pitch and front-servo load; stop when load splits front/rear acceptably. Do NOT drop the front instead (breaches the tibia envelope — in-plane reach floor 0.1079 m at q3=−2.2).
  4. Persistent after trim: the CoM is genuinely off — Q-014 (exact CoM measurement, battery installed) is the real fix; set base_link inertial origin, re-derive trim.
- ROOT CAUSES SEEN: rear-heavy load distribution (observed in sim, fixed by D-016: +15 % distance, −42 % yaw drift, −98 % lateral).
- FIX: trim ladder above; sim precedent says raise-the-rear, never drop-the-front.
- PREVENTION: Q-014 CoM measurement (open) makes the trim principled instead of empirical; until then re-verify trim after ANY mass relocation.

### A19 — Teensy and lidar (or bench boards) swap /dev names across boots
- SYMPTOM: `real.launch.py device:=/dev/ttyACM0` grabs the lidar (or vice versa) after a reboot/replug.
- CHECK: `ls -l /dev/serial/by-id/` — every USB serial device has a stable by-id path.
- FIX NOW: launch with the by-id path: `device:=/dev/serial/by-id/usb-Teensyduino_*`.
- FIX PROPERLY: udev rule per device (electronics phase folder installs these):
  `SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="teensy", MODE="0666"` (Teensy 4.1 VID/PID — verify with `udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct'` before trusting), analogous rule → `SYMLINK+="lidar"` for the lidar's UART bridge. Then `device:=/dev/teensy` forever.
- PREVENTION: no script may reference a bare ACM/USB index once more than one serial device exists on the robot.

### A20 — Gazebo GUI renders black, or robot meshes invisible **[hit]**
- SYMPTOM: GUI window black; or world visible but BARQ's meshes missing (collisions work).
- CHECK IN ORDER:
  1. Black GUI on this Jetson (Tegra GL): launched with `--render-engine-gui ogre`? (sim.launch.py `gui:=true` already does.)
  2. Invisible meshes: `IGN_GAZEBO_RESOURCE_PATH` must point at the description share's parent so `package://barq_description/...` resolves (sim.launch.py sets it; hand-started `ign gazebo` does not).
  3. Headless captures black ≠ broken: gnome-screenshot/xwd can't capture direct-GL — verify via the VNC client or `ign model -m barq --pose`.
- ROOT CAUSES SEEN: ogre2 GUI black on Tegra; unset resource path; screenshot tooling lying.
- FIX/PREVENTION: always start the GUI through sim.launch.py `gui:=true`; judge headless runs by pose/telemetry, not screenshots.

### A21 — Two ROS stacks cross-polluting (ghost controllers, duplicate /controller_manager) **[hit]**
- SYMPTOM: nodes/services from a stack you didn't start; spawners grabbing the wrong manager.
- CHECK: `docker ps` — more than one ROS container on host network without domain isolation?
- ROOT CAUSE SEEN: `--network host` containers share ONE DDS graph; running the RViz demo + sim simultaneously cross-polluted /controller_manager.
- FIX: one stack at a time; if two are genuinely needed, isolate: `export ROS_DOMAIN_ID=<n>` (same n in every shell of that stack; integration_pty.py already self-isolates on 42).
- PREVENTION: crib's exec pattern includes the domain note; check `docker ps` before every launch.

### A22 — Results inconsistent run-to-run; a config "randomly" collapses **[hit — mid-stride teleport fiasco]**
- SYMPTOM: same gait config scores 60 % one run, −10 % the next.
- CHECK: was the robot reset to a CLEAN state between runs? (Sim: fresh spawn or teleport while STOPPED at stance. Hw: power-cycle + settle at stance.)
- ROOT CAUSE SEEN: teleporting mid-stride carried gait phase/momentum into the "new" run; every config looked broken.
- FIX: fresh spawn (or stop → teleport → settle ≥ 2 s) per measured run; on hardware, fresh power-cycle per measured run for anything near a gate.
- PREVENTION: Protocol §2 "between-run state is part of the experiment"; sim_walk_metric runs assume it.

### A23 — `mode:=real` controllers won't activate, or die mid-run (stale link)
- SYMPTOM: ros2_control_node up but activation fails; or controllers stop with an ERROR after running fine.
- CHECK IN ORDER:
  1. Is STATE flowing? BarqSystem REQUIRES a live STATE frame to activate (starts from measured pose, anti-lurch). No STATE in `state_timeout_ms` (300 ms) at activation ⇒ refuses; mid-run staleness ⇒ deliberate ERROR stop.
  2. Firmware alive? Teensy LED: solid = driven, blink = idle/deadman. No LED = power/flash problem.
  3. Link sanity without ros2_control: `ros2 run barq_hw integration_pty.py` (emulator) proves the host stack; then PING the real device (protocol PING/PONG — firmware phase folder bench script).
  4. Deadman latched? STATE fault bit3 set ⇒ the Teensy stopped hearing CMDs (100 Hz writer stalled — check container RT caps, Q-009: `--cap-add=SYS_NICE --ulimit rtprio=99`).
  5. Wrong device (A19), wrong baud/port busy (A12).
- ROOT CAUSES EXPECTED: no/stale STATE (by design), USB CDC stall under Orin load (risk D-R7), device mixup.
- FIX: per rung — the refusal IS the safety feature; fix the link, don't lengthen the timeout first.
- PREVENTION: integration_pty green before every hardware session (regression spine).

### A24 — Robot collapses then snaps to pose at spawn/power-on **[hit in sim; hw twin predicted]**
- SYMPTOM: sim: spawn-drop + violent first command. Hw twin: power-on lurch when controllers start.
- CHECK: (sim) URDF `initial_value`s = stance? spawn z = 0.17 (≈3 cm drop onto stance legs)? (hw) Did activation read the measured pose first (A23 #1)?
- ROOT CAUSE SEEN: no initial joint values + spawn 0.25 m (pre-D-014); fixed by exact-stance initial_values + spawn 0.17.
- FIX/PREVENTION: sim is fixed in-repo. Hardware: BarqSystem's activation-from-measured-pose is the equivalent; never bypass it; first torque-on always in the cradle (Protocol §3).

### A25 — SLAM maps NOTHING despite scans flowing **[hit]**
- SYMPTOM: /scan healthy, slam_toolbox quiet, map empty/never appears.
- CHECK IN ORDER:
  1. Features: is there actual geometry within range? An empty world/featureless corridor maps nothing (hit in sim with the empty world).
  2. Motion thresholds: slam_toolbox only updates after `minimum_travel_distance: 0.2` m / `minimum_travel_heading: 0.3` rad — drive a bit.
  3. Range window: scan returns inside [min 0.3, max 20.0] m (`barq_slam.yaml`)? A wall at 0.2 m is invisible.
  4. Then A5 (TF chain) if updates still don't come.
- FIX/PREVENTION: map in feature-rich spaces; for the real lidar keep `min_laser_range` above the robot's own body radius so it doesn't map itself.
