# P5-01 — Lidar: purchase decision, bench bringup, mount, scan QA

> Phase P5 · verified against repo @ 4ea53a0

## Objective
Buy the right 2D lidar inside the ₹24,000 ceiling, bring its driver up on the Jetson, mount it
so the scan plane is level at stance (replicating the URDF's −4.5° counter-wedge), and prove a
clean, accurate, self-hit-free `/scan` on the topic/frame the whole stack already expects
(`/scan`, frame `laser`). The sim was built lidar-first for exactly this moment: the URDF sensor
block (`barq_description/urdf/barq.urdf.xacro` lines ~379–401) IS the STL-27L spec — buy the
primary candidate and nothing in the software model changes.

Full selection analysis (specs, mass argument, A2M12 rejection, mitigation theory):
`docs/research/2026-06-11-lidar-selection.md`. This file does not repeat it; it executes it.

## Prerequisites
- P4 complete enough that the robot can stand at stance (G5.2 needs the real stance pitch).
  Bench bringup (steps 2–5) needs ZERO robot — do it the day the unit arrives, or before, on
  the Jetson alone.
- `barq:dev` container builds; `~/barq_ws` colcon workspace healthy.
- Tape measure, a flat cardboard target ≥ 0.3 m wide, two identical rigid boxes ≥ 0.30 m tall,
  a stiff card, masking tape, digital kitchen scale (for the unit's real mass).
- Budget authority for ≤ ₹24,000.

---

## 1. Purchase decision gate (run this BEFORE ordering — it binds the money)

Decision ladder (from the research doc, Q-015). Work top-down; buy the first row that passes
ALL its criteria. **Do not buy the RPLidar A2M12** regardless of price — rejected for mass
class (190 g on a 2.45 kg robot), see research doc §1.

| Rank | Unit | Buy if ALL of: |
|---|---|---|
| A | **LDROBOT STL-27L** | landed price (unit + shipping + duties) ≤ ₹24,000 · deliverable to India ≤ 3 weeks · listing confirms UART/USB adapter included (or add a CP2102-class USB-UART board to the order) |
| B | **LDROBOT LD19 / D500 kit** | STL-27L failed a criterion · landed price ≤ ₹24,000 · ≤ 3 weeks |
| C | Park P5 purchase | both failed → re-quote in 2 weeks; meanwhile advance P6-01/02 or P7 prep (protocol §5 parallel graph) |

Checklist at order time (record answers in the research log — this is a TBD-table):

| TBD | How to fill |
|---|---|
| STL-27L landed price in INR | quote from ≥ 2 sources (LDROBOT store/AliExpress/robu.in/Amazon.in); include customs estimate |
| Lead time to India | seller-stated + 1 week margin |
| Adapter board included? chip type? | listing photos/Q&A; expect CP2102-class (verify — drives the udev rule in §2) |
| LD19/D500 landed price | same procedure, only if rank A fails |

Reference prices from the research doc (2026-06-11, ~$142 ≈ ₹13k for STL-27L, ~$80–110 for
LD19/D500) — REVERIFY at order time; do not treat them as current.

### 1.1 If the fallback (LD19/D500) is bought — model deltas
The sim model and SLAM config are written to the STL-27L. A fallback purchase changes three
numbers in two files (then rebuild + re-run a sim SLAM lap as regression):

| What | File · key | STL-27L (current) | LD19/D500 value |
|---|---|---|---|
| Samples per rev @ 10 Hz | `barq_description/urdf/barq.urdf.xacro` → `<sensor name="lidar2d">` → `<samples>` | 2160 | 450 (4.5 kHz / 10 Hz) |
| Max range | same block → `<range><max>` | 25.0 | 12.0 |
| SLAM range cap | `barq_bringup/config/barq_slam.yaml` → `max_laser_range` | 20.0 | 11.5 (keep below sensor max) |
| Mass | URDF `laser` link `<mass>` | 0.047 | weigh the real unit; update if it differs by > 5 g |

`barq_nav2.yaml` costmap `raytrace_max_range: 12.0` / `obstacle_max_range: 10.0` already fit
inside an LD19's 12 m — no nav2 change needed for either unit.

Whichever unit arrives: **weigh it with its bracket** and put the real mass in the URDF laser
link (the research doc's rule: RL trains on the robot that exists). Q-014's CoM measurement
must happen with the lidar installed.

---

## 2. Jetson serial prep (do BEFORE the unit arrives — 10 minutes, zero risk)

Both the lidar (USB-UART adapter, `/dev/ttyUSB*`) and the Teensy (USB CDC, `/dev/ttyACM*`) are
USB-serial. Enumeration order is not stable across boots → fixed names via udev now.

1. Remove the verified landmine (research doc §2: brltty 6.4-4ubuntu3 hijacks CP2102 on THIS
   Jetson) and join dialout, **on the host**:
   ```bash
   sudo apt remove brltty
   sudo usermod -aG dialout barq     # re-login (new SSH session) to take effect
   ```
2. Discover identity attributes of each device (host, device plugged in). For the Teensy
   (already owned — do it today):
   ```bash
   udevadm info -a -n /dev/ttyACM0 | grep -E 'idVendor|idProduct|serial' | head -8
   ```
   Expected for Teensy 4.1 USB Serial: idVendor `16c0`, idProduct `0483` — VERIFY, do not
   copy blind. Repeat for the lidar adapter when it arrives (`-n /dev/ttyUSB0`); expected
   CP2102-class `10c4:ea60` per the research doc — VERIFY (record the `serial` attr too if
   present; it disambiguates if a second CP2102 ever appears).
3. Write `/etc/udev/rules.d/99-barq-serial.rules` on the host (substitute the verified IDs):
   ```
   SUBSYSTEM=="tty", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0483", SYMLINK+="barq_teensy", MODE="0666"
   SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="barq_lidar", MODE="0666"
   ```
   If lidar adapter and any other device share a VID:PID, add `ATTRS{serial}=="<value>"` to
   pin the right one.
4. Reload + verify:
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ls -l /dev/barq_teensy /dev/barq_lidar    # symlinks -> ttyACM0 / ttyUSB0
   ```
5. Container passthrough: add to the `barq:dev` run line of any container that needs them:
   `--device /dev/barq_lidar --device /dev/barq_teensy`
   (plus the standing `-v /dev/shm:/dev/shm`). From now on launches use the stable names:
   `real.launch.py device:=/dev/barq_teensy`, lidar driver `port: /dev/barq_lidar`.

---

## 3. Driver bringup (bench: lidar on the desk, robot not involved)

Candidate ROS 2 Humble drivers, in preference order — **verify each is still maintained and
lists your exact unit before building** (check the repo's README + last-commit date; this is
the one purchase-dependent software unknown):

| Rank | Package | Notes |
|---|---|---|
| A | `ldlidar_stl_ros2` (github.com/ldrobotSensorTeam/ldlidar_stl_ros2) | LDROBOT official, STL/LD series incl. STL-27L + LD19; source build |
| B | `ldrobot-lidar-ros2` (github.com/Myzhar/ldrobot-lidar-ros2) | community, lifecycle nodes; confirm STL-27L support before choosing |
| C | Raw-serial custom node | LD protocol is publicly documented; ~150 lines of parsing; last resort only |

Switch criteria A→B: driver won't build or won't stream after 2 attempts or 2 hours. B→C: same,
plus a check of open issues for your unit (if the protocol changed, C may genuinely be faster).

1. Build in the container as a workspace package:
   ```bash
   cd ~/barq_ws/src && git clone <chosen driver repo>
   cd ~/barq_ws && colcon build --packages-select <driver_pkg> && source install/setup.bash
   ```
2. Configure (param names vary by driver — set these five things, whatever they are called):
   - port: `/dev/barq_lidar`
   - baud: STL-27L expected 921600; LD19/D500 expected 230400 — **verify against the vendor
     README**, wrong baud = silent no-data
   - `frame_id: laser` · topic `/scan` (zero-config parity with the sim and with
     `barq_slam.yaml`/`barq_nav2.yaml` — non-negotiable)
   - output must be REP-103 (CCW-positive, x-forward). Check `angle_increment > 0` and that an
     object moved to the lidar's left appears at positive bearing.
3. Power note: bench bringup over the USB adapter is fine. Before robot mounting, MEASURE draw
   with an inline USB power meter (TBD row below). The research doc's BEC rule was written for
   the A2M12's 1.5 A spin-up; the LD units are far lighter loads — measure, don't assume:
   steady < 0.5 A and a clean 10-min soak (no re-enumeration in `dmesg -w`) → powering from the
   Jetson USB port is acceptable; otherwise dedicated 5 V BEC ≥ 2 A, common GND, adapter
   carries TX/RX only.
4. Stream it:
   ```bash
   timeout -k 2 30 ros2 topic hz /scan --window 50
   timeout -k 2 10 ros2 topic echo /scan --once --field angle_increment
   timeout -k 2 10 ros2 topic echo /scan --once --field ranges | grep -c '^- '   # sample count
   ```

### 3.1 Bench scan QA (this is gate G5.1)
1. **Rate**: `timeout -k 2 120 ros2 topic hz /scan --window 100` → mean ≥ 9.5 Hz over 2 min.
2. **Sample count**: per-scan ranges count = 2160 (STL-27L) or ~450 (LD19/D500), stable across
   10 consecutive scans. (Some drivers emit variable counts per packet assembly — if so, note
   it and confirm slam_toolbox copes, but flag in the log.)
3. **Range accuracy**: flat cardboard target, tape-measured at 1.00 / 3.00 / 5.00 m from the
   lidar's optical center, lidar level on the desk. Read the range at the target's bearing:
   ```bash
   timeout -k 2 20 python3 - <<'EOF'
   import rclpy, math
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan
   rclpy.init(); n = Node('qa')
   def cb(m):
       i = round((0.0 - m.angle_min) / m.angle_increment)   # bearing 0 = straight ahead
       print('range @0deg:', m.ranges[i]); rclpy.shutdown()
   n.create_subscription(LaserScan, '/scan', cb, 1); rclpy.spin(n)
   EOF
   ```
   Bar: |measured − tape| ≤ 3 cm at each distance, 5-sample spread ≤ 2 cm.
4. **Endurance**: 10 min continuous streaming, `ros2 topic hz` checked at start/middle/end,
   zero gaps > 1 s (watch the driver log and `dmesg` for USB resets).

**G5.1 — bench scan: 10 min continuous, mean rate ≥ 9.5 Hz, expected sample count, range error
≤ ±3 cm at 1/3/5 m, zero dropouts > 1 s.** Fail → ladder §7.

---

## 4. Mount fabrication — replicate the URDF wedge exactly

The URDF is the spec. `laser_joint` (barq.urdf.xacro line ~43):
`xyz = −0.04 0 0.066` (m, from base_link origin: 4 cm aft of center, on the deck centerline,
6.6 cm up to the OPTICAL CENTER of the unit) · `rpy = 0 −0.0785 0` (**−4.5° pitch**).

(The research doc §5 says z = 0.075 — superseded; the URDF's 0.066 is what the sim flew and
what physics validated. Code wins, per roadmap README.)

Why the wedge: the D-016 stance trim sits the body 4.5° nose-down; the mount pitches the lidar
4.5° nose-UP relative to the deck so the scan plane comes out level. Sim-verified: odom→laser
rotation ≈ 0.001 rad at stance (research log §2b). The wedge's thick edge goes at the FRONT of
the lidar. **Before fabricating, sanity-check the sign in RViz** (launch
`visualize.launch.py`, orbit the laser link: its top face must tilt nose-up relative to the
deck).

1. Fabricate a bracket (printed PLA or filed aluminum, TBD): 4.5° ± 1° wedge, thick edge
   forward, positioning the unit's optical center at (−0.04, 0, 0.066) from base_link origin.
   Aft-of-center is deliberate — the nose deck is reserved for a future depth camera (research
   doc §3); do not "improve" the position forward.
2. Keep a clear 360° optical horizon at the scan plane: no standoffs, wiring, or fasteners at
   optical-center height. Route the cable down and forward under the deck lip.
3. Mount rigidly (research doc mitigation #2: scan-matching hates a wobbling sensor). No foam.
4. Weigh unit + bracket + fasteners → update URDF laser `<mass>` if it differs > 5 g from 47 g.
5. **Do NOT add a `static_transform_publisher` for base_link→laser.** On hardware,
   `robot_state_publisher` (already in `real.launch.py`) publishes it from the URDF's fixed
   joint. Verify it is there and correct:
   ```bash
   timeout -k 2 10 ros2 run tf2_ros tf2_echo base_link laser
   # expect: translation [-0.040, 0.000, 0.066], rotation rpy [0, -0.0785, 0]
   ```
   **Failure-tree symptom — doubled TF**: scan visibly jitters/rotates against the robot,
   `TF_REPEATED_DATA` warnings spam the logs, or tf2_echo alternates between two values →
   somebody added a duplicate static publisher for `laser`. Remove it; robot_state_publisher
   is the only legitimate source.

### 4.1 Scan-plane level verification (gate G5.2 — on the standing robot)
The beam is 905 nm-class IR (invisible); find the plane by occlusion, not by eye.

1. Robot at stance on a flat floor, lidar streaming, torque ON, robot stationary.
2. Tape marks on the floor at exactly 1.00 m forward of and 1.00 m behind the lidar center.
3. At the FRONT mark, hold the stiff card vertically and lower it from ~0.4 m height until it
   first appears in the scan at bearing ~0° (watch ranges around index for 0° with the §3.1
   snippet, or RViz over VNC — setup use only). Tape-measure the card's bottom edge height
   off the floor. Repeat 3×, take the median → `h_front`.
4. Repeat at the REAR mark (bearing ±180°, index ≈ first/last sample) → `h_rear`.
5. Level check: with the plane at ~0.21 m (0.066 above base origin + ~0.142 stance height),
   ±1° of pitch = ±1.7 cm at 1 m. Bars:
   - |h_front − h_rear| ≤ 3.5 cm (total tilt ≤ 1° over the 2 m baseline), and
   - both within ±3 cm of the lidar's own optical-center height (tape-measured at the unit).
6. Out of spec → shim the bracket (each 1° ≈ 0.7 mm shim across a 40 mm bracket footprint),
   re-measure. If the robot's stance pitch itself is off, fix THAT first (P4 trim, D-016) —
   the wedge is matched to −4.5° stance, not to whatever the robot happens to do.

**G5.2 — scan plane level at stance within ±1° by the fore/aft card procedure.**

---

## 5. Self-hit check and the scan filter chain (gate G5.3)

Geometry (from `barq_description/config/robot_params.yaml`: body box 0.258 × 0.117): from the
lidar at x = −0.04, the body's tail edge is 0.089 m away, rear corners ≈ 0.107 m, side edges
0.0585 m, nose edge 0.169 m. The SIM never saw self-hits because the sim sensor's min range is
0.15 m — it physically cannot return the tail/sides. The real unit reports returns well below
that, and the real deck carries the bracket, wiring, and P1's electronics. Expect self-hits.
`barq_slam.yaml` is protected by `min_laser_range: 0.3`, but the **nav2 costmap obstacle layers
read `/scan` raw** — a self-hit becomes a permanent lethal obstacle ON the robot.

1. Detect: robot standing, stationary, in the middle of a ≥ 2 m clear area:
   ```bash
   timeout -k 2 20 python3 - <<'EOF'
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan
   rclpy.init(); n = Node('selfhit')
   def cb(m):
       hits = [(i, r) for i, r in enumerate(m.ranges) if m.range_min < r < 0.30]
       print(len(hits), 'returns inside 0.30 m:', hits[:20]); rclpy.shutdown()
   n.create_subscription(LaserScan, '/scan', cb, 1); rclpy.spin(n)
   EOF
   ```
   Zero hits → record that fact and skip to the gate (leave the filter unbuilt but keep this
   spec). Any hits → build the filter chain.
2. Filter chain — `ros-humble-laser-filters` (apt; named in the research doc §3 — verify
   installable in the container, else source-build). Config
   `barq_bringup/config/barq_scan_filter.yaml` (verify exact param schema against the
   laser_filters README for your installed version — the shape below is the spec):
   ```yaml
   scan_to_scan_filter_chain:
     ros__parameters:
       filter1:
         name: body_crop
         type: laser_filters/LaserScanBoxFilter
         params:
           box_frame: base_link
           min_x: -0.16     # body ±0.129 + 3 cm margin
           max_x:  0.20     # extra nose margin: future depth camera lives there
           min_y: -0.09     # body ±0.0585 + 3 cm margin
           max_y:  0.09
           min_z: -0.05
           max_z:  0.30
           invert: false    # remove points INSIDE the box
   ```
   Node: `laser_filters/scan_to_scan_filter_chain`, input `/scan`, output `/scan_filtered`.
   It needs the base_link←laser TF — robot_state_publisher provides it (§4.5).
3. Re-point consumers at the filtered topic (only when the filter is in use):
   - `barq_bringup/config/barq_slam.yaml` → `scan_topic: /scan_filtered`
   - `barq_bringup/config/barq_nav2.yaml` → BOTH costmaps' `obstacle_layer.scan.topic:
     /scan_filtered` (local ~line 142, global ~line 174)
   Sim keeps raw `/scan` (it has no self-hits); make these hardware-side copies per P5-02 §1,
   don't edit the sim's files in place.
4. Verify: re-run the step-1 snippet against `/scan_filtered` → zero returns inside 0.30 m,
   AND a real obstacle placed at 0.5 m still appears (the filter must not eat the world).

**G5.3 — 60 s of `/scan_filtered` (or `/scan` if step 1 found zero hits) with zero returns
inside the crop box while a 0.5 m test obstacle remains visible.**

---

## 6. Acceptance gates (P5-01)
| Gate | Bar |
|---|---|
| **G5.1** | Bench: 10 min continuous scan, ≥ 9.5 Hz mean, expected samples/scan, ±3 cm at 1/3/5 m, no dropout > 1 s |
| **G5.2** | On robot at stance: scan plane level within ±1° (fore/aft card procedure) |
| **G5.3** | Filtered scan clean: zero self-hits inside the crop box, real obstacles intact |

## 7. Fallback ladders
- **Purchase**: A STL-27L → B LD19/D500 (apply §1.1 deltas) → C park + re-quote in 2 weeks,
  advance P6/P7 in parallel. Never the A2M12.
- **Driver**: A official → B Myzhar → C custom serial parser. Switch after 2 failed attempts
  or 2 h per rung. At every rung the acceptance is the same G5.1.
- **No /dev/ttyUSB appears**: 1) `lsusb` — adapter enumerated at all? try another cable/port;
  2) `dmesg | tail -30` — if the device node appears then vanishes, brltty is back (§2.1);
  3) module present? `lsmod | grep cp210x` (research doc verified it ships on this Jetson);
  4) swap-test the adapter on the Mac before blaming the Jetson.
- **Self-hits**: A box filter (§5) → B raise the bracket +10 mm (then z becomes 0.076 — update
  the URDF and re-run G5.2) → C driver-side min-range if supported. Switch A→B only if hits
  come from above the filterable plane geometry (e.g., a tall mast in view).

## 8. Rollback
Everything here is additive. The robot's P4 state (walking, no lidar) is untouched by not
launching the driver. udev rules: `sudo rm /etc/udev/rules.d/99-barq-serial.rules && sudo
udevadm control --reload-rules`. URDF mass/sensor edits are single-line git reverts. The
purchase itself is the only non-reversible step — that is why §1 is a gate.

## 9. Artifacts → docs/05_RESEARCH_LOG.md
Order-time TBD table (§1) · udevadm identity attrs as measured · chosen driver + commit hash ·
G5.1 numbers (rate, count, range error at 1/3/5 m) · measured unit mass and URDF delta ·
power-draw measurement + chosen power source · h_front/h_rear and final shim state ·
self-hit count before/after filter. One log entry per session, per standing practice.

## Escape hatch
If no acceptable 2D lidar can be purchased inside budget after two quote rounds, P5 continues
in sim only (the full SLAM/nav2 stack is already proven there) and the team decides between
waiting and re-scoping perception around a nose depth camera (research doc §3 — substantial
rework: depthimage_to_laserscan, narrower FOV; treat as a new decision, not a fallback). If
stuck ≥ 2 sessions on driver/QA: write it up in docs/04_OPEN_QUESTIONS.md with everything
measured, park, advance P6 (protocol §4–5).
