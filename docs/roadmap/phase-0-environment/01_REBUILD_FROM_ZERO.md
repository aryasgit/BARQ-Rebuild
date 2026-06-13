# P0-01 — Rebuild the dev environment from zero (bare Jetson → sim walks)

> Phase P0 · verified against repo @ 4ea53a0

## Objective
From a bare (or replacement) Jetson Orin Nano, reach the proven Stage-4 state with nothing but
this file and the repo: JetPack 6 flashed, `barq:dev` Docker image built, workspace built
**including the vendored `external/gz_ros2_control` overlay**, PlatformIO on the host,
remote GUI viewing working — and all three validation gates green (G0.1 sim walks,
G0.2 `integration_pty.py` 9/9, G0.3 firmware builds + native tests 6/6).

Nothing here needs robot hardware. A USB keyboard/HDMI monitor OR a second computer
(Mac/PC as SSH client) is enough.

## Prerequisites
- Jetson Orin Nano (developer kit) + power supply; NVMe SSD or ≥64 GB SD card
  (the image stack needs **≥ 30 GB free** after OS).
- For flashing route A: an x86 host PC with Ubuntu 20.04/22.04 (bare metal strongly
  preferred — USB re-enumeration during flash is flaky in VMs) + USB-C data cable.
- Network access (the Docker base image alone is >10 GB of pulls).
- A GitHub account with read access to `aryasgit/BARQ-Rebuild` and an SSH key registered.
- This roadmap printed or on a second screen — the Jetson will reboot several times.

## Procedure

### 1. Flash JetPack 6 (L4T r36.4.x)
The container base is `dustynv/ros:humble-desktop-l4t-r36.4.0` — the host L4T **major version
must be r36** (JetPack 6). Check after flash:

```bash
cat /etc/nv_tegra_release        # expect: # R36 (release), REVISION: 4.x ...
```

**Route A (primary) — SDK Manager from the x86 host PC:**
1. Install NVIDIA SDK Manager on the host PC (nvidia.com/sdkmanager, needs free NVIDIA login).
2. Put the Jetson in recovery mode: power off, short the **FC REC** pin to **GND** on the
   button header (see the devkit user guide silk-screen), connect USB-C to the host, power on.
   `lsusb` on the host must show an NVIDIA Corp APX device — if not, you are not in recovery.
3. In SDK Manager select Jetson Orin Nano, **JetPack 6.x (L4T r36.4.x)**, target storage =
   your NVMe/SD. Flash OS + (optionally) the CUDA runtime components. 30–60 min.
4. First boot: create user `barq`, hostname `barq` (so `barq.local` mDNS works as documented).

**Route B (fallback) — SD card image, no host PC needed:**
1. Download the "Jetson Orin Nano Developer Kit" SD image for **JetPack 6** from NVIDIA's
   Getting Started page; write with balenaEtcher.
2. **Landmine:** units shipped with older QSPI boot firmware (r35.x era) will NOT boot a
   JetPack 6 SD card until the QSPI firmware is updated. Symptom: black screen / boot loop.
   Fix path is documented by NVIDIA (boot an older JetPack 5.1.3 image once and apply the
   firmware update, or use SDK Manager once) — **verify the current procedure on NVIDIA's
   page at execution time**; it has changed between releases.

### 2. Host prep (user, serial, hygiene)
```bash
sudo apt update
sudo apt remove -y brltty            # brltty hijacks CP2102 USB-UARTs; /dev/ttyUSB0 vanishes
sudo usermod -aG dialout barq        # serial device access without sudo
sudo apt install -y git curl htop python3-pip openssh-server avahi-daemon
# re-login (or reboot) so the group takes effect; verify:
id | grep -o dialout                 # expect: dialout
```
The brltty removal is not optional: version 6.4 (stock on Ubuntu 22.04) grabs CP2102/CH340
adapters within seconds of plug-in. This burned us once already
(`docs/research/2026-06-11-lidar-selection.md` §2).

### 3. Docker + NVIDIA runtime
JetPack usually preinstalls Docker with the nvidia runtime. Verify; install only if missing:
```bash
sudo docker info | grep -i runtimes        # expect the list to include: nvidia
# if missing:
sudo apt install -y docker.io nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo usermod -aG docker barq && newgrp docker
# smoke test (pulls a small image):
docker run --rm --runtime nvidia ubuntu:22.04 echo nvidia-runtime-ok
```
Expected: `nvidia-runtime-ok`. If `--runtime nvidia` errors with "unknown runtime", the
toolkit configure step did not write `/etc/docker/daemon.json` — check that file by hand.

### 4. Clone the repo (SSH over 443 — port 22 is blocked on our network)
```bash
ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519 -N ""    # if no key yet
cat ~/.ssh/id_ed25519.pub                            # add to the GitHub account (Settings→SSH keys)
```
Create `~/.ssh/config` with EXACTLY this (verbatim from the working machine):
```
Host github.com
    HostName ssh.github.com
    Port 443
    User git
    IdentityFile ~/.ssh/id_ed25519
```
```bash
ssh -T git@github.com        # expect: "Hi <user>! You've successfully authenticated..."
mkdir -p ~/barq_ws
git clone git@github.com:aryasgit/BARQ-Rebuild.git ~/barq_ws/src
cd ~/barq_ws/src && git checkout stage-2 && git log --oneline -1
```
Workspace layout matters: the container mounts `~/barq_ws` at `/root/barq_ws`, so the repo
must be at `~/barq_ws/src` (colcon's `src/`), not anywhere else.

### 5. Build the `barq:dev` image (with its known landmines)
```bash
cd ~/barq_ws/src
docker build -t barq:dev .            # 30–90 min: >10 GB base pull + apt layers
```
Expected final line: `Successfully tagged barq:dev`. Landmines, in the order you'd hit them:

| Landmine | What the failing log looks like | Fix |
|---|---|---|
| ROS apt key expired | `apt update` layer: `EXPKEYSIG F42ED6FBAB17C654` | The Dockerfile's first RUN already refreshes the key from rosdistro; if it recurs, re-pull that key URL — do not `apt-key adv` hacks |
| OpenCV conflict (Gazebo layer) | `dpkg: error processing archive .../libopencv-dev_*.deb ... trying to overwrite '/usr/include/opencv4/...' which is also in package opencv-dev` | Cause: pulling `ros-gz` metapackage or any package that drags Ubuntu `libopencv-dev` into the dustynv CUDA-OpenCV base. The Dockerfile deliberately installs only `ros-humble-ros-gz-sim` + `ros-humble-ros-gz-bridge`. Do not "fix" by installing the metapackage |
| Same conflict (nav2 layer) | identical `trying to overwrite '/usr/include/opencv4/...'` | nav2's layer already uses `--no-install-recommends -o Dpkg::Options::="--force-overwrite"` (headers only; nothing in the image compiles against OpenCV — safe). Keep both flags |
| Disk full mid-build | `no space left on device` in any layer | Need ≥30 GB free; `docker system prune` and retry — layers cache, the build resumes fast |

### 6. Build the workspace — vendored plugin FIRST
```bash
docker run --runtime nvidia -it --rm --network host --shm-size=8g \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev
# ---- inside the container ----
cd /root/barq_ws
GZ_VERSION=fortress colcon build --packages-select gz_ros2_control   # the BARQ-patched overlay
colcon build --symlink-install --packages-skip gz_ros2_control
source install/setup.bash
ls install/gz_ros2_control/lib/ | grep ign_ros2_control   # expect: libign_ros2_control-system.so
cd /root/barq_ws/src/barq_control && python3 -m pytest test/          # expect: 20 passed
```
**Why the overlay is non-negotiable:** fresh clones that skip the `gz_ros2_control` build get a
sim that *silently* falls back to the soft `/opt` plugin (servo stiffness k=10/s instead of the
ST3215-true 60/s — see `external/gz_ros2_control/PROVENANCE.md` and D-018). Everything still
"works"; the walk is just mush and every number you tune is wrong. G0.1's ~60% realized speed
doubles as the detector: the soft plugin lands far below it.

Note on `-v /dev/shm:/dev/shm`: FastDDS same-host transport is shared memory. A single
container works without it; **any** cross-container ROS (e.g. a separate RViz container)
silently delivers no data without it. Mount it always; it costs nothing. (The convenience
scripts `~/run_barq.sh` / `~/run_barq_gui.sh` predate this rule — add the mount if you
recreate them.)

### 7. PlatformIO on the HOST (not in Docker)
The Teensy toolchain and USB flashing live on the Jetson host (`~/.local/bin/pio`), never in
the container:
```bash
python3 -m pip install --user platformio
~/.local/bin/pio --version                       # expect: PlatformIO Core 6.x
# Teensy upload udev rules (needed by P3 flash day; harmless to install now):
curl -fsSL https://www.pjrc.com/teensy/00-teensy.rules | sudo tee /etc/udev/rules.d/00-teensy.rules
sudo udevadm control --reload-rules
```
First `pio run -e teensy41` downloads the ARM toolchain (~1 GB) — do it while you have network.

### 8. Remote GUI viewing (VNC from the Mac/any VNC client)
The Jetson is headless; GUIs render on a forced X display, viewed over VNC. The helper
scripts (`~/fix_display.sh`, `~/setup_vnc.sh`, `~/run_barq_gui.sh`) live in the home
directory, NOT the repo — on a fresh machine recreate the essentials:

1. Force the display "connected" — `/etc/X11/xorg.conf` (back up any original first):
```
Section "Device"
    Identifier "Tegra"
    Driver     "nvidia"
    Option     "AllowEmptyInitialConfiguration" "true"
    Option     "ConnectedMonitor" "DP-0"
EndSection
```
   then `sudo systemctl restart gdm`. Display comes up at 1024×768 (higher modes need a real
   EDID — `xrandr --addmode` is rejected; live with it).
2. VNC server on that display:
```bash
sudo apt install -y x11vnc
x11vnc -storepasswd                # writes ~/.vnc/passwd
x11vnc -display :0 -auth /run/user/1000/gdm/Xauthority -rfbauth ~/.vnc/passwd \
       -forever -shared -rfbport 5900 &
```
3. Client: macOS Finder → ⌘K → `vnc://barq.local:5900` (or the IP). Any VNC viewer works.
4. GUI containers additionally need the X mounts (`-e DISPLAY=:0`, `-v /tmp/.X11-unix`,
   xauth cookie) — the full recipe is in `~/run_barq_gui.sh` on the original machine and in
   memory doc `barq-remote-viz`; Gazebo GUI on Tegra needs `--render-engine-gui ogre`
   (already wired into `sim.launch.py`).

Trust the VNC client, not host screenshots: direct-GL windows screenshot black.

### 9. Validation gates

**G0.1 — the sim walks (ST3215-true).** Headless, one container, one DDS stack:
```bash
docker run --runtime nvidia -d --name barq_sim --network host --shm-size=8g \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev \
  bash -lc "source /opt/ros/humble/setup.bash && source /root/barq_ws/install/setup.bash && \
            ros2 launch barq_bringup sim.launch.py gait:=true gui:=false"
sleep 45 && docker logs barq_sim --tail 20      # check the LOG, not the exit code
docker exec barq_sim bash -lc "source /opt/ros/humble/setup.bash && \
  source /root/barq_ws/install/setup.bash && \
  timeout -k 2 90 python3 /root/barq_ws/src/diagnostics/sim_walk_metric.py --vx 0.15 --duration 10"
docker stop barq_sim
```
Expected one-line output (numbers will vary a little):
```
WALK vx=0.15 T=10.0s  fwd=+0.9xxm lat=±0.0xxm yaw=±0.xxxrad  speed=0.09xm/s (~60% of commanded)
```
**PASS band: 50–70% of commanded** at the default `gait_duty 0.6` (D-019). Below ~45% with a
clean log almost always means the vendored plugin didn't load — redo step 6 and re-check
`libign_ros2_control-system.so`. Always wrap `ros2`/metric invocations in `timeout -k 2 <s>`;
the ros2 CLI swallows SIGTERM under load and wedges the exec.

**G0.2 — full hardware-interface stack on the firmware emulator, 9/9.**
Self-isolates on `ROS_DOMAIN_ID=42`, but the clean habit is: nothing else running.
```bash
docker run --runtime nvidia --rm --network host --shm-size=8g \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev \
  bash -lc "source /opt/ros/humble/setup.bash && source /root/barq_ws/install/setup.bash && \
            timeout -k 2 180 ros2 run barq_hw integration_pty.py"
```
Expected final line: `9/9 checks passed` (exit 0 = drop-in ready; on hardware day only
`device:=` changes — D-020).

**G0.3 — firmware: native tests 6/6 + Teensy build SUCCESS.** On the HOST:
```bash
cd ~/barq_ws/src/barq_firmware
~/.local/bin/pio test -e native        # expect: 6 Tests 0 Failures 0 Ignored → [PASSED]
~/.local/bin/pio run  -e teensy41      # expect: ... [SUCCESS] (no board attached needed)
```

## Acceptance gates
| Gate | Bar | Measured by |
|---|---|---|
| **G0.1** | sim walk realizes **50–70%** of `vx 0.15` (duty 0.6), straight, clean log | `diagnostics/sim_walk_metric.py` WALK line |
| **G0.2** | `integration_pty.py` prints **9/9 checks passed**, exit 0 | step 9 command |
| **G0.3** | `pio test -e native` **6/6** AND `pio run -e teensy41` **SUCCESS** | host PlatformIO |

All three green = P0-01 done; record artifacts and move to `02_BOM_PROCUREMENT.md`.

## Fallback ladders

**Ladder 1 — JetPack flash fails**
- **A:** SDK Manager, bare-metal x86 host (retry once with a different USB-C cable/port —
  cables cause most failures).
- **B:** SD-card image route (step 1 route B), QSPI caveat included.
- **C:** Different host PC entirely; as a last resort SDK Manager in a VM with USB
  passthrough pinned to the NVIDIA APX device (known flaky — expect retries).
- *Switch criteria:* 2 failed flash attempts on a route → next rung. All three fail →
  the module is suspect; try the known-good Jetson, file the RMA.

**Ladder 2 — `docker build` fails on apt conflicts**
- **A:** Build the Dockerfile exactly as committed (the landmines table above covers the
  known failure signatures).
- **B:** If a NEW package conflict appears (upstream repo drift): split the failing RUN
  into its own layer, add `--no-install-recommends`, then if (and only if) the collision is
  header-/doc-only, extend `-o Dpkg::Options::="--force-overwrite"` to that layer. Record
  which package needed it in the research log.
- **C:** Version-pin the failing packages to the versions in a known-good image
  (`docker run --rm <good-image> dpkg -l | grep <pkg>` on any surviving machine/registry
  copy), or export/import a saved `barq:dev` tarball (`docker save`/`docker load`) if one
  exists on the Mac or a backup disk.
- *Switch criteria:* same layer fails twice with the same signature after a cache-clean
  retry → next rung. Burned > 1 day → rung C.

**Ladder 3 — no Mac / no VNC client available**
- **A:** VNC route as in step 8 (any laptop with a VNC viewer works, not just a Mac).
- **B:** HDMI/DP monitor + USB keyboard directly on the Jetson — skip step 8 entirely; the
  desktop runs locally, no xorg.conf forcing needed (revert it if applied).
- **C:** Fully headless: `gui:=false` everywhere, verify pose with
  `ign model -m barq --pose`, judge walking by `sim_walk_metric.py` numbers, RViz never.
  Everything in P0 is gate-able without a single GUI.
- *Switch criteria:* immediate — pick whichever hardware you actually have.

## Rollback
Nothing in this file is destructive past the flash itself.
- Bad workspace build: `rm -rf ~/barq_ws/{build,install,log}` (host paths) and redo step 6.
- Bad image: `docker rmi barq:dev` and rebuild (base layers stay cached).
- Bad xorg.conf (display dead even on a real monitor): restore the backup
  (`sudo mv /etc/X11/xorg.conf.backup /etc/X11/xorg.conf`; on the original machine the
  backup is `/etc/X11/xorg.conf.barq-backup`), restart gdm.
- Bad flash: re-flash (route A). The repo + GitHub remote is the only state that matters;
  the Jetson is cattle, not a pet.

## Artifacts to record (→ `docs/05_RESEARCH_LOG.md`)
- `cat /etc/nv_tegra_release` output and JetPack version flashed; flash route used (A/B).
- `git log --oneline -1` of the checked-out commit.
- Image build wall-time + any landmine hit (which one, log signature, fix applied).
- The literal G0.1 `WALK ...` line, G0.2 `9/9 checks passed` line, G0.3 test/build tails.
- Any deviation from this file — then UPDATE this file in the same commit.

## If this entire phase approach fails
(Jetson dead/unobtainable, dustynv base image gone from registries, no flash host at all.)
The repo is the asset, not the Jetson. The sim stack is architecture-portable: on any x86
Ubuntu 22.04 machine, build a substitute image `FROM ros:humble` + the same apt list (drop
the dustynv/CUDA specifics; the OpenCV landmines disappear too) + Gazebo Fortress from
packages.osrfoundation.org, then run steps 6 and 9 unchanged — G0.1/G0.2/G0.3 are all
CPU-honest (the firmware tests never needed the Jetson at all). You lose: CUDA, and the
deploy target itself. That keeps P0–P3 software work and all of P6 sim work alive while a
replacement Orin Nano is sourced; P3 hardware-in-loop and beyond resume on the new module
with this same file.
