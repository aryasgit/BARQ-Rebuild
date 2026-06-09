# BARQ — Changelog

Dated log of concrete repo changes. Newest first.

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
