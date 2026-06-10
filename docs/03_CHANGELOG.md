# BARQ — Changelog

Dated log of concrete repo changes. Newest first.

---
## 2026-06-11 — Stage 2E COMPLETE: BARQ walks in Gazebo physics, head-first
Multi-day bring-up, three phases:

**1. Gazebo Fortress infrastructure.** Dockerfile: OSRF repo + slim package set
(`ros-gz-sim`, `ros-gz-bridge`, `ign-ros2-control` — NOT the `ros-gz` metapackage: its
demos/image extras pull Ubuntu libopencv-dev which collides with the dustynv image's CUDA
opencv-dev; first build failed exactly there). `barq.urdf.xacro` mode:=gazebo branch
(ign_ros2_control hardware + system plugin reading ros2_controllers.yaml). Collisions
reduced to body box + 4 foot spheres (r=0.012 at the kinematic foot point) — mesh collisions
are slow/unstable for contacts. Offline `barq_world.sdf` (no Fuel). `sim.launch.py`:
gz server (+optional GUI), spawn from /robot_description, /clock bridge, spawners, gait:=true.
GOTCHA: with --network host all containers share one DDS graph — running the RViz demo and
the sim simultaneously cross-polluted /controller_manager etc. One stack at a time.

**2. Adversarial review (25-agent workflow): 3 confirmed findings, 18 refuted.**
(a) startup collapse-then-snap: no initial joint values + spawn 0.25 m; (b) the idealized
leg model was off by up to 3.4 cm at the FRONT feet (knee x-offset +-0.01744 front/rear,
femur 10.7deg in-plane angle, 0.0324 lateral ankle offset all folded into fake "lengths" —
front/rear ASYMMETRIC, support polygon shifted, stand height 11 mm off); (c) zero swing
clearance (step 0.012 = sphere r, apex 0.1 rad from the tibia clamp).
Bonus correction: Humble ign_ros2_control applies position commands as a stiff velocity
loop (JointVelocityCmd = error x update_rate), not literal teleports; initial values DO
use JointPositionReset.

**3. Exact kinematics + fixes (D-014).** `leg_kinematics.py`: fk_exact/ik_exact modelling
the URDF chain exactly (q1-invariant x-offsets, femur in-plane L2P/A2, combined lateral
LAT=0.0754692; URDF +q2 tilts the femur toward -X — the legacy model had this sign
MIRRORED, the root of the direction confusion). Verified against a rotation-matrix
composition of the raw URDF origins to <1e-12 (test_exact_kinematics.py); legacy model kept
but marked not-for-control. ik_node + gait use the exact model; neutral foot = knee-x
forward/back of hip (symmetric support polygon). Gait stand 0.13 / step 0.02 (real
clearance; tibia apex -2.18 within -2.2). URDF: initial_value on all 12 joints = exact
stance (0, 1.047531, -1.928768); spawn z 0.17. gait_planner gains `forward_sign` param.

**Physics results** (headless, ign model --pose):
- Settle: x=-0.000001, z=0.141779 vs predicted 0.142 (0.2 mm!), level, zero transient
  (was: -3.2 cm startup lurch + 11 mm height anomaly — both were the model error).
- Walk cmd_vel +x: **-0.376 m in ~8 s toward -X = HEAD-FIRST, the correct direction**
  (was +X/tail-first; cause was the asymmetric foot placement + scuffing, not the mapping).
  forward_sign=-1 confirmed as default. ~0.047 m/s realized of 0.12 commanded (open-loop
  slip — tuning headroom for later).
19 unit tests pass. Stray ws-root demo files cleaned.

---
## 2026-06-10 — Deep crouch: tibia judgment limit -2.2, stand_height 0.115; first push
Team confirmed the ST3215s are **360-deg servos — no hard mechanical stop**; all joint limits are
design judgment (resolves Q-001's premise). Aryaman wanted a much deeper crouch (more femur, more
tibia fold, narrow). Changes:
- Tibia limit -1.57 -> **-2.2 rad (~126deg)** everywhere: URDF ankle joints (x4), ros2_control
  command interfaces (x4), robot_params servo map (x4), ik_node clamp (now asymmetric [-2.2, 0]).
- stand_height 0.16 -> **0.115 m (~28% lower)**; ik_node static stance matches. New floor:
  min 2-link reach at q3=-2.2 is ~0.094 m -> stand-step must stay >= ~0.103 m.
- Regression test bounds updated to [-2.2, 0]; **14 tests pass**; xacro 0 stderr; check_urdf OK.
Live: femur +0.93..+0.98, tibia -1.96..-2.08, diagonals intact. Flag: verify no link collision at
full fold on the physical build (D-012).
Git: `stage-2` pushed to origin (first push) — SSH over 443; all commits authored Aryaman Gupta.

---
## 2026-06-10 — Gait reversed (head-first) + crouched stance
Aryaman (after fold fix): gait stepped toward the tail end; wanted travel toward the head, and a lower
body for stability. Two changes, both at the gait layer (URDF/IK/gait math untouched):
1) `gait_planner_node` now maps cmd_vel (robot-centric, +x = head-first) into body axes by negating
   linear x,y (yaw unchanged) — reverses the traversal arc. Q-012 documents the frame story.
2) Crouch: defaults stand_height 0.18 -> 0.16, step_height 0.03 -> 0.012 (node + gait.py). Constraint
   honored: stand-step >= ~0.147 m, else the swing apex demands tibia beyond -1.57 (leg geometry).
New regression test `test_default_gait_stays_within_tibia_range` (full default cycle, tibia within
[-1.571, 0]) — **14 tests pass**. Live: knees +0.56..+0.75, ankles ~-1.36..-1.55, diagonals intact.

---
## 2026-06-10 — Knee-bend branch flipped: legs now fold FORWARD (Q-010 resolved)
Aryaman (visual check over VNC): all legs folded backward — tibia closed toward the body to the rear;
femur+tibia needed the mirrored pose ("45 -> 135", both joints, all legs). That is exactly the other
analytical IK elbow branch. Fix: default `knee_bend` +1 -> **-1** in `leg_kinematics.ik_leg` and the
`ik_node` param. Stance mirrors (knees -0.73 -> +0.73, ankles +1.52 -> -1.52); foot positions identical
(FK-verified). New unit test `test_forward_fold_branch` (same foot point, q3 <= 0) — **13 tests pass**.
Corroboration: servo config in robot_params has tibia range **[-1.571, 0]** — the -1 branch is the only
one the real hardware can reach (strong evidence toward Q-001's one-directional tibia).
Live-verified walking: knees +0.45..+0.70, ankles -1.03..-1.34 (inside the servo range), trot diagonals
preserved. NOTE: direction-of-travel / which end is +X is deliberately deferred (Aryaman: "that's for
later") — only the fold direction changed; gait paths and URDF untouched.
(An earlier frame-flip attempt (f4cd735) was reverted by request — branch reset to 2982b06 first.)

---
## 2026-06-10 — Stage 2D: trot gait planner (verified — BARQ walks)
Added `barq_control/gait.py`: open-loop trot foot-trajectory generator (diagonal pairs FL+RR / FR+RL,
50% duty, swing arc + stance sweep; step size scales with cmd velocity; neutral stance at zero cmd).
6 unit tests pass (zero-cmd stance, periodicity, diagonal sync, swing lift, step scaling, all targets
IK-reachable). Added `gait_planner_node.py`: /cmd_vel (Twist) -> /foot_targets @ 50 Hz, gait params as
ROS params. `control.launch.py` gains `gait:=true` (implies ik via a PythonExpression OR-condition).
setup.py console_script + geometry_msgs dep.

Lint: brought barq_control fully clean — import ordering (ament style), D213 docstrings, removed unused
imports, line lengths. All 12 tests green (4 IK + 6 gait + flake8 + pep257).

Verified live (headless): /cmd_vel x=0.15 -> all 6 nodes up, /joint_states cycles with correct diagonal
pairing (FL_knee==RR_knee, FR_knee==RL_knee). Tuned defaults stand_height=0.18 / step_height=0.03 so the
ankle stays within +/-1.57 during swing (at 0.16 the legs were too bent to lift without clamping).

Foundation audit (pre-2D): clean build from scratch (5/5), interface counts all 12, barq_hardware (C++)
and barq_sim correctly deferred to Stage 4 / 2E.

---
## 2026-06-10 — Stage 2C: analytical IK node (verified)
Added `barq_control/leg_kinematics.py`: idealized 3-DOF leg (clean link lengths from robot_params;
coxa about X, femur/tibia about Y) with forward kinematics and closed-form analytical IK. Unit tests
`test/test_ik.py` verify FK<->IK round-trip to 1e-9 across the workspace + neutral/unreachable cases
(4 pass). Fixed an angle-wrap (`+acos` branch pushed q1 past pi -> normalize outputs to (-pi, pi]).
Added `ik_node.py`: loads geometry, streams a default stance, subscribes `/foot_targets` (12 foot xyz,
body frame, FL/FR/RL/RR), publishes 12 joint positions to the position controller @ 50 Hz, clamped to
joint limits. `control.launch.py` gains `ik:=true`. Wired barq_control setup.py (console_script) +
package.xml (rclpy/std_msgs/ament_index_python/yaml/barq_description deps).

Verified live (headless): IK node @ 50 Hz; stance hips=0, knees=-0.73, ankles=1.52 (symmetric across
4 legs); commanding feet 0.19 m below hips straightened the legs (knees -0.39, ankles 0.82). Knee-bend
branch = +1 (verify visual direction in RViz, Q-010).

---
## 2026-06-10 — Stage 2B: ros2_control mock loop (verified)
Renamed `barq.urdf` -> `barq.urdf.xacro` (added `xmlns:xacro`, a `mode` arg) and appended a
`<ros2_control>` system block: `mock_components/GenericSystem` (mode=mock), all 12 joints with a
position command interface + position/velocity state interfaces. Added
`barq_bringup/config/ros2_controllers.yaml` (joint_state_broadcaster + joint_group_position_controller
over all 12 joints) and `control.launch.py`. Pointed `visualize.launch.py` at the `.xacro`; `setup.py`
installs `config/`; `package.xml` gains controller_manager / joint_state_broadcaster /
position_controllers / ros2controlcli deps.

**Gotcha fixed:** macro params named `min`/`max` shadow xacro builtins -> "redefining global symbol"
warnings on **stderr** -> the launch `Command` substitution treats any stderr as failure and aborts
before a single node starts. Renamed params to `lower`/`upper` (xacro now emits 0 bytes stderr).

**Verified headless (mode=mock):** 4 nodes up; `joint_state_broadcaster` +
`joint_group_position_controller` both ACTIVE; publishing `{hips=0, knees=+0.3, ankles=-0.6}` to
`/joint_group_position_controller/commands` -> `/joint_states` reflects it. Benign: ros2_control_node
can't set FIFO RT scheduling under Docker (Q-009).

---
## 2026-06-09 — Stage 2A DONE: BARQ rendering in RViz over VNC
After `fix_display.sh`, `:0` came up at 1024x768 with DP-0 forced connected (no more black).
`xrandr --addmode` for 1600x900 was rejected by the NVIDIA driver (no EDID) -> stayed at 1024x768
(usable). Resized `barq.rviz` to fit (1024x728, right dock hidden) and relaunched the `barq_rviz`
container. Confirmed via host screenshot: **RViz shows the full BARQ model** — body, 4 legs,
coxa/femur/tibia, curved feet, TF triads, grid; OpenGl 4.6. Stage 2A visual check PASSED.
(For 1600x900 later: supply a real EDID blob to CustomEDID in the xorg.conf.)

---
## 2026-06-09 — Headless display resolution fix (fix_display.sh)
Root-caused the black screen: Orin is headless (DP-0/DP-1 disconnected, NVIDIA Tegra driver,
`AllowEmptyInitialConfiguration` -> 640x480 default, no EDID on system). `xrandr --fb` enlarges the
buffer but nothing scans it out -> black. Added `~/fix_display.sh` (host): backs up
`/etc/X11/xorg.conf`, writes a config that forces DP-0 `ConnectedMonitor` + a cvt `1600x900_60`
Modeline + `AllowNonEdidModes`, then restarts gdm (autologin=barq recreates the :0 session). Safe:
falls back to 640x480 if the mode is rejected; revert via the backup. Pending: user runs
`sudo ~/fix_display.sh`, then Claude restarts x11vnc + relaunches RViz. No repo files changed.

---
## 2026-06-09 — RViz running over VNC; headless display notes
Brought Stage 2A visualization up live over VNC. Launched the visualize stack in a detached container
(`barq_rviz`) rendering to `:0`: robot_state_publisher loaded all 13 segments; rviz2 got **OpenGl 4.6**
(GPU-accelerated on the Tegra); jsp_gui up. Gotchas (for future reference):
- Headless `:0` defaults to **640x480** (no monitor) -> RViz (1200x800) opened mostly off-screen.
  `xrandr --fb 1600x900` enlarges it without sudo, but GNOME/mutter may need a VNC reconnect to repaint.
- `gnome-screenshot`/`xwd` can't reliably capture direct-GL windows headlessly (return black) — verify
  via the VNC client, not host screenshots.
- Two RViz instances were running (user's interactive + the detached one); removed the duplicate.
- WARN: root link `base_link` has inertia; KDL ignores it (harmless for RViz; matters for Gazebo — Q-008).
Tooling only; no repo files changed.

---
## 2026-06-09 — VNC path for remote display (Route B)
Route A (ssh -Y to Mac) failed: Mac `$DISPLAY` empty (XQuartz not serving), so chose Route B.
Probe: desktop session is X11 + active; Xorg `-auth /run/user/1000/gdm/Xauthority` (owned by `barq`,
so no root needed to run x11vnc); port 5900 free; passwordless sudo NOT available (install must be
user-run). Added `~/setup_vnc.sh` (host): installs x11vnc via sudo, sets a VNC password, `xhost
+SI:localuser:root` so the container can draw on :0, starts `x11vnc -display :0` on :5900 (-bg).
Tightened `run_barq_gui.sh` to use the gdm Xauthority when `DISPLAY=:0`. View from Mac:
`vnc://barq.local:5900`. No repo files changed.

---
## 2026-06-09 — Remote display tooling (run_barq_gui.sh)
Added `~/run_barq_gui.sh` (host, alongside `run_barq.sh`): same container launch but with X11
passthrough — DISPLAY + wildcard `xauth` cookie merge + `/tmp/.X11-unix` mount + `QT_X11_NO_MITSHM`,
and `LIBGL_ALWAYS_INDIRECT` auto-set (direct for local `:0`, indirect for forwarded). Lets RViz/GUIs
show on the Mac via SSH X11 forward (Route A) or render on the Jetson `:0` GPU for VNC (Route B).
Host facts confirmed: live Xorg on `:0`, sshd `X11Forwarding yes`, `xauth` installed. No repo files changed.

---
## 2026-06-09 — Stage 2A wired up; URDF integrated & verified
Received artifacts validated and the visualization path made to actually work.

**Received (via barq-channel from the Mac):**
- `barq_description/urdf/barq.urdf` — 13 links, 12 revolute joints.
- `barq_description/meshes/{base_link,coxa,mid,mid_rev,foot,foot_rev}.dae` — all non-zero,
  `mid`/`mid_rev` and `foot`/`foot_rev` confirmed distinct (genuine mirrors).

**Changed:**
- `urdf/barq.urdf`: 24 mesh refs `filename="X.dae"` -> `package://barq_description/meshes/X.dae` (D-002).
- `barq_description/CMakeLists.txt`: added `install(DIRECTORY urdf meshes config ...)` (D-003).
- `config/robot_params.yaml`: placeholder geometry -> real values from URDF (D-005).
- `barq_bringup/launch/visualize.launch.py`: new — robot_state_publisher + joint_state_publisher_gui
  + rviz2, `gui:=true|false` arg (D-004).
- `barq_bringup/rviz/barq.rviz`: new — Grid + RobotModel(/robot_description) + TF, fixed frame base_link.
- `barq_bringup/setup.py`: install `launch/` and `rviz/`.
- `barq_bringup/package.xml`: added exec_depends (barq_description, robot_state_publisher,
  joint_state_publisher_gui, rviz2, xacro, launch, launch_ros).
- `.gitignore`: ignore `build/ install/ log/`.
- `docs/`: created the support-docs system (this folder).

**Verified (headless, inside `barq:dev`):**
- `check_urdf` — valid tree (base_link -> 4x coxa->femur->tibia).
- `colcon build --symlink-install` — 5/5 packages OK (only setuptools easy_install deprecation
  warnings on the Python packages).
- All 6 `package://` mesh targets exist in the installed share dir.
- `launch/visualize.launch.py` + `rviz/barq.rviz` present in installed share.
- `xacro` parse of the *installed* URDF — 13 links, 12 joints.

**Not yet verified:** actual RViz render (needs an X display).
**Not committed:** awaiting git policy (Q-003).
