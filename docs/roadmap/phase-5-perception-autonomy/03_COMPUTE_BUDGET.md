# P5-03 — Orin Nano compute budget for the on-robot stack

> Phase P5 · verified against repo @ 0e5ddaf

## Objective
Keep the full mission stack (ros2_control + estimator + gait + lidar driver + slam_toolbox +
nav2) inside the Orin Nano's envelope with margin, so the sim-era failure — **action goals
silently lost under load** (research log §2d) — never happens on hardware. Measure first,
then apply a fixed mitigation ladder. Two gates: CPU headroom with zero lost goals (G5.7) and
thermal steady state (G5.8).

Context that makes this tractable: the two biggest sim-era loads (physics + lidar rendering,
GUI rendering) DO NOT EXIST on the real robot, and the Teensy split already isolates
hard-real-time from Jetson load (changelog 2026-06-11). The Jetson's remaining job is
perception + planning + the 50-ish Hz control/estimator path. The budget is about protecting
that path.

## Prerequisites
- P5-02 stack launches (`robot.launch.py gait:=true slam:=true nav:=true`).
- Robot can run safely on the cradle for measurement configs (a) and (b); floor + spotter for (c).
- An inline battery wattmeter (XT60) and/or the P1-02 INA260 telemetry path for draw numbers.
- Host access for `tegrastats`, `nvpmodel`, `jetson_clocks` (they run on the HOST, not in the
  container).

---

## 1. Baseline measurement (fill the TBD table — never guess these)

Three configurations, 10 min each, robot active (walking on cradle for a/b, slow mission for c):

- **(a) control only**: `real.launch.py gait:=true` + estimator, walking on cradle.
- **(b) + lidar + SLAM**: `robot.launch.py gait:=true slam:=true`, teleop creep on cradle.
- **(c) + nav2**: full mission stack, a real G5.5-style mission on the floor.

Procedure per configuration:
1. Host, in parallel with the run (note `tegrastats` needs sudo for some fields; plain works):
   ```bash
   tegrastats --interval 1000 --logfile /tmp/tegrastats_<cfg>.log &
   top -b -d 5 -n 120 -o %CPU | grep -E 'ros2|slam|nav2|component|lidar|estimator|gait' \
       > /tmp/top_<cfg>.log &
   ```
2. After 10 min: `kill %1 %2`. Extract from the tegrastats log: RAM used/total, per-core CPU %
   (the `CPU [..%@freq,..]` block — also watch the FREQUENCY: cores pinned below max = DVFS or
   thermal throttling), `soc`/`cpu` temperature fields, `SWAP` usage, GPU (`GR3D_FREQ` —
   should be ~0 %: **the real robot renders nothing; a nonzero GPU load during a mission means
   a renderer leaked in**).
3. Battery draw: read the inline wattmeter at minute 5 (steady), or average the INA260 bus
   current telemetry if P1-02's `/power` path is live.
4. Lost-goal check (config c only): the mission must show the §P5-02-4.3 telemetry chain
   (accepted → "Begin navigating" → "Goal succeeded").

| TBD | cfg (a) control | cfg (b) +slam | cfg (c) +nav2 | Procedure |
|---|---|---|---|---|
| CPU % total (mean / peak) | — | — | — | tegrastats log, 60 s rolling mean over all cores |
| Per-core peak % | — | — | — | same (a single pinned core at 100 % matters more than the mean) |
| RAM (MB used) | — | — | — | tegrastats RAM field at minute 10 |
| Swap (MB) | — | — | — | tegrastats SWAP (nonzero swap under mission load = automatic ladder trigger) |
| SoC / CPU temp (°C @ 10 min) | — | — | — | tegrastats temp block |
| Battery draw (W) | — | — | — | wattmeter / INA260, minute 5 |
| /scan rate under load (Hz) | n/a | — | — | `timeout -k 2 30 ros2 topic hz /scan --window 100` (sim degraded 9.7 → ~7 Hz under full load — research log §2b; hardware bar: ≥ 9.5) |
| Goals lost in 5 sends | n/a | n/a | — | mission_runner × 5, count not-accepted / no-"Begin navigating" |

## 2. Power-mode selection

JetPack 6 mode tables vary by L4T point release — **verify on THIS Jetson, don't trust a
table from the internet**:
```bash
sudo nvpmodel -q --verbose        # current mode
grep -E 'POWER_MODEL|NAME' /etc/nvpmodel.conf   # all available modes (expect 15W, 7W,
                                                # possibly 25W "MAXN SUPER" on JP 6.x Super releases)
```
Fill a mode table: run baseline config (c) once per available mode and record CPU %, temps,
battery draw, and lost-goal count.

| TBD | per mode (one column per mode found) | Procedure |
|---|---|---|
| Mode name / cap (W) | — | nvpmodel.conf |
| cfg (c) CPU mean % | — | §1 procedure |
| cfg (c) SoC temp @ 10 min | — | §1 |
| Battery draw (W) | — | §1 |
| Goals lost / 5 | — | §1 |

Selection rule: choose the LOWEST power mode that passes G5.7 — battery runtime and thermal
margin are mission resources (the 5200 mAh 4S budget is shared with 12 servos). If only the
max mode passes, take it and note the draw delta in the log. `sudo jetson_clocks` (pins
clocks, disables DVFS) is a measurement tool and a last-resort stabilizer — if it changes
G5.7 from fail to pass, the real problem is scheduling latency, not throughput; prefer ladder
rung B.

Mode changes: `sudo nvpmodel -m <n>` (may prompt reboot). Record mode in every research-log
entry — results are not comparable across modes.

## 3. Thermal (gate G5.8)

1. 20 min continuous mission-class load (config c, repeated missions back-to-back), ambient
   noted (matters in summer).
2. Log temps the whole time (tegrastats already does). Also check throttling directly:
   ```bash
   cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq   # sample at min 0/10/20
   ```
   Falling frequency at constant load = thermal throttling even if temps look "fine".
3. **Passive vs fan: TBD by this soak.** If SoC < 85 °C passive at 20 min with stable clocks →
   passive is enough, close the TBD. Else fit the fan (5 V, from the P1 power tree — NOT the
   servo rail), repeat the soak, record both curves.

| TBD | Value | Procedure |
|---|---|---|
| SoC temp @ 20 min, passive | — | soak above |
| Clock stability passive (throttle y/n) | — | scaling_cur_freq samples |
| SoC temp @ 20 min, fan | — | only if passive fails |

## 4. Mitigation ladder (apply IN ORDER when starved; re-measure a 5-mission block after each rung)

Starvation symptoms (sim-proven, transfer): action goals accepted-but-lost or never accepted;
TF delays (`transform_tolerance` violations in controller/costmap logs, "Could not transform"
spam); `/scan` rate sagging below 9.5 Hz; swap nonzero. Trigger the ladder on ANY of these or
on G5.7 numbers.

**A — shed visualization and logging (free, always first).**
No RViz, no Gazebo, no renderers on the Jetson during missions — `top` must show no
`rviz2`/`gzserver`/render processes, tegrastats GR3D ~0 %. (RViz-over-VNC renders ON the
Jetson — setup sessions only; mission viewing is Foxglove on the Mac or nothing, P5-02 §4.1.)
rosbag only selective, never `-a` (costmaps/maps are heavy):
```bash
timeout -k 2 <missionlen> ros2 bag record -o /data/bags/m_$(date +%H%M) \
  /scan /odom_est /cmd_vel /tf /tf_static /joint_states
```
Drop launch `output='screen'` verbosity where it floods (estimator/gait debug prints).

**B — process priorities (control path must win contention).**
Q-009: the container currently CANNOT set RT scheduling ("Operation not permitted"). The real
robot's container run line gains:
```
--cap-add=SYS_NICE --ulimit rtprio=99
```
(spec these as deltas to the standing `barq:dev` run command, alongside `-v /dev/shm:/dev/shm`
and the §P5-01 `--device` flags). Then, stack running:
```bash
pgrep -af 'ros2_control_node|gait_planner|state_estimator|ik_node'   # collect PIDs
sudo chrt -f -p 30 <pid_ros2_control_node>
sudo chrt -f -p 20 <pid_state_estimator>
sudo chrt -f -p 20 <pid_gait_planner>; sudo chrt -f -p 20 <pid_ik_node>
chrt -p <pid>    # verify each: SCHED_FIFO
```
Rules: ONLY the control path gets RT — never slam_toolbox/nav2 (they must lose contention,
not win it). Keep priorities ≤ 30 (well under kernel-critical threads). Safety net if an RT
task runaway starves everything: the Teensy's 200 ms firmware deadman torques the robot off —
that is by design (protocol §3). Priorities reset on process restart: script them into the
mission pre-flight, or adopt a launch prefix later.

**C — rate reductions (exact keys + safe floors; one change at a time, measured).**

| File | Key | Current | Reduced | Floor / why |
|---|---|---|---|---|
| barq_slam_real.yaml | `map_update_interval` | 2.0 | 5.0 | 10.0 — map freshness only; SLAM still corrects TF |
| barq_slam_real.yaml | `transform_publish_period` | 0.05 | 0.1 | 0.1 — must stay ≪ nav2 `transform_tolerance` 0.3 |
| barq_slam_real.yaml | `minimum_travel_distance` | 0.2 | 0.3 | 0.3 — fewer scan-match solves at 0.10 m/s |
| barq_nav2_real.yaml | `controller_frequency` | 10.0 | 5.0 | 5.0 — at 0.10 m/s that is 2 cm per control cycle |
| barq_nav2_real.yaml | local costmap `update_frequency` | 4.0 | 2.0 | 2.0 — 5 cm of travel between updates |
| barq_nav2_real.yaml | local costmap `publish_frequency` | 2.0 | 1.0 | publish is for viewers only |
| barq_nav2_real.yaml | global costmap `update_frequency` | 1.0 | 0.5 | 0.5 |
| barq_nav2_real.yaml | `smoothing_frequency` (velocity_smoother) | 10.0 | 5.0 | 5.0 — must stay ≥ controller_frequency/2 |
| barq_nav2_real.yaml | `expected_planner_frequency` | 1.0 | 0.5 | 0.5 — RPP replans off the existing path |

Also available at this rung: run in **localization mode** on a finished map (P5-02 §2.2) —
no graph growth, materially cheaper than mapping mode.

**D — offload viz/monitoring to the Mac.**
Live viewing moves off-Jetson entirely: `foxglove_bridge` on the robot (one websocket;
subscribe ONLY to what you watch — each subscription costs serialization) → Foxglove app on
the Mac. Or zero live viz: selective bag (rung A) + offline review. VNC remains for
non-mission setup work.

**E — LAST resort: SLAM on the Mac over wifi DDS.**
Architecture change; everything before it is configuration. Why last (document of caveats):
- FastDDS same-host shared memory (`/dev/shm`) is **irrelevant across hosts** — the transport
  becomes UDPv4 over wifi; the sim-era /dev/shm rule does not help here.
- Discovery: multicast over wifi is unreliable → needs an explicit
  `FASTRTPS_DEFAULT_PROFILES_FILE` with `initialPeers` (unicast, both hosts) or a FastDDS
  Discovery Server (`fastdds discovery -i 0` on the Jetson + `ROS_DISCOVERY_SERVER` on both).
  Same `ROS_DOMAIN_ID` on both ends; every other stack on the LAN must use a different one.
- **Latency risk to TF is the killer**: slam_toolbox on the Mac would publish map→odom across
  wifi; jitter spikes > `transform_tolerance` (0.3 s) stall costmaps and the controller
  mid-mission. Scan bandwidth itself is trivial (2160 × 4 B × 10 Hz ≈ 90 KB/s) — it is the
  TF/timing path that breaks.
- The Mac needs a linux/arm64 ROS 2 Humble container; Docker Desktop on macOS lacks true host
  networking (verify on current version) — the Discovery Server route is effectively
  mandatory there.
- Switch criterion: only after A–D are all applied AND G5.7 still fails, and then only for
  SLAM (control/estimator/nav2 stay on the robot — nav2's control loop over wifi is not an
  option). If E is ever adopted: missions require a wifi-loss drill (kill the link mid-run →
  robot must stop or continue safely on the last map→odom; measure, log, decide).

Reversal: every rung is reversible (A/D: process hygiene; B: restart resets priorities, drop
the docker flags; C: revert the table keys; E: relaunch SLAM on-robot).

## 5. Acceptance gates (P5-03)
| Gate | Bar |
|---|---|
| **G5.7** | Full mission stack: total CPU ≤ 80 % sustained (60 s rolling mean, tegrastats) AND no single core pinned at 100 % for > 10 s AND **zero lost action goals across 5 consecutive missions** (lost = not accepted, or accepted with no "Begin navigating" within 10 s) AND swap = 0 |
| **G5.8** | Thermal steady state: SoC < 85 °C with stable CPU clocks through a 20 min continuous mission session, in the CHOSEN power mode and cooling configuration |

## 6. Rollback
Power mode: `sudo nvpmodel -m <previous>`. Ladder rungs: §4 reversal notes. Container flag
deltas live in the documented run command — removing them returns to the Q-009 status quo
(no RT, warns harmlessly). Nothing here touches sim configs or the control stack's code.

## 7. Artifacts → docs/05_RESEARCH_LOG.md
The filled §1/§2/§3 TBD tables (per power mode) · chosen mode + cooling and why · every
ladder rung applied with before/after numbers (one change at a time — the rung you DIDN'T
need is also a result) · G5.7 5-mission table · the as-built docker run command for the
mission container. Close or update Q-009 in docs/04_OPEN_QUESTIONS.md when rung B lands.

## Escape hatch
If G5.7 cannot be met even at rung E: fix the mission definition instead of the platform —
map first (G5.4), then run all missions in localization mode at the 0.10 m/s ceiling with
rates at the §4C floors; that is a smaller, honest mission envelope, and it still satisfies
the P5 exit gate (autonomous A→B indoors). Log the ceiling as a standing limitation in
docs/04_OPEN_QUESTIONS.md, and revisit only if P6/P7 demand more (e.g., a Jetson swap is a
team budget decision, not a tuning step). If stuck ≥ 2 sessions: protocol §4 — park with the
measurements attached and advance a parallel phase.
