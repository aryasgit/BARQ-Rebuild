# Appendix B — Master Acceptance-Gate Index

> Appendix · verified against repo @ 0e5ddaf

Phase docs were written in parallel with this index, so **each phase folder is authoritative for
its own G-numbers, exact bars, and fallback ladders**. This appendix gives (1) the gate THEMES
each phase must close — derived from the roadmap README phase table and the Protocol's definition
of done — so you can sanity-check that no theme was dropped, and (2) the **anti-regression
spine**: the project-wide test set that must stay green through every phase.

The finish line (Protocol §6, verbatim themes): (a) untethered repeatable walking under
`/cmd_vel`, (b) autonomous indoor A→B with lidar SLAM + nav2 self-recovering, (c) RL policy
beats classical OR the no-RL bypass ladder adopted ≥ rung 2, (d) the field runbook in real use.

## 1. Gate themes by phase

### P0 — `phase-0-environment/` (rebuild dev env; BOM; bench)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| Image + workspace build | docker build & `colcon build` **log tails** (not exit codes) | all packages build; known apt conflicts handled per repo Dockerfile | phase folder G# |
| Patched-plugin canary | `ros2 param get /gz_ros2_control position_proportional_gain` with sim up | reads **0.6** (0.1 = soft fallback → failure tree A10) | phase folder G# |
| Sim walk reproduced | `sim_walk_metric.py --vx 0.15` fresh spawn, duty 0.6 | ≥ 55 % realized (D-019 envelope 57–62 %) | phase folder G# |
| Firmware toolchain | `pio test -e native`; `pio run -e teensy41` | 6/6 tests; teensy41 builds | phase folder G# |
| Stage-4 stack, zero hw | `ros2 run barq_hw integration_pty.py` | 9/9 | phase folder G# |
| BOM/procurement | parts list vs owned inventory | everything for P1–P2 ordered or in hand | `02_BOM_PROCUREMENT.md` |

### P1 — `phase-1-power-electronics/` (4S power tree, 12 V buck, Teensy, 4 buses, BNO085, INA260)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| Battery verified | chemistry (4.20 vs 4.35 V/cell) BEFORE first charge; floor discipline | charger profile matches pack; floor 13.6 V on record | phase folder G# |
| Buck proven under load | output V + temp at incremental dummy loads; ripple | holds ≥ ~11.7 V at worst-case rail draw (TBD table in folder) | `01_POWER_TREE.md` G# |
| Fusing + master switch | every rail fused; torque-off one action away | per power-tree drawing | phase folder G# |
| Monitoring | INA260 readings vs DMM | agreement within folder's tolerance; ≤ 15 A/unit respected | `02_MONITORING_INA260.md` G# |
| Teensy link | PING/PONG over USB CDC; STATE at 100 Hz | protocol v1 green on real silicon | phase folder G# |
| IMU + buses on bench | BNO085 streaming; each driver board scans servos | every board talks on the bench | phase folder G# |
| Brownout rehearsal | supervised sag test near floor | documented behavior, thresholds recorded (→ E registry TBDs) | phase folder G# |

### P2 — `phase-2-*` calibration & assembly (per-servo bench calib, IDs, zero pose, limits, D-012)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| 12/12 bench-certified | `st3215_diag.py` scan/status/sweep per servo | all healthy per diagnostics/README criteria; serials logged | phase folder G# |
| IDs + directions | `plan` map (0–11) vs physical labels; direction signs | matches `robot_params.yaml servos:` exactly | phase folder G# |
| Zero calibration | calibrate-mid pre-assembly; assembled-neutral residuals | **`zero_offset` rows in robot_params filled (no longer 0.0)** | phase folder G# |
| D-012 fold check | physical link collision at tibia −2.2 (torque off, then slow) | no collision through full fold, or limit re-judged + recorded | phase folder G# |
| Assembly at zero pose | leg-by-leg `monitor` while hand-moving | smooth, no dropouts, correct signs end-to-end | phase folder G# |

### P3 — `phase-3-*` firmware integration (fill stubs; HIL; bench ID)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| Stubs filled | `servo_bus_*`, `imu_read`, `power_read` in `loop_core.*` | real telemetry in STATE (pos/vel/load, quat, vbus) | phase folder G# |
| Loop timing | 500 Hz superloop + 100 Hz STATE on real Teensy | rates held under full bus load | phase folder G# |
| **Drop-in contract** | integration-test pattern on **/dev/ttyACM0** | the P0 emulator checks pass against the flashed Teensy | phase folder G# |
| Deadman on hardware | kill the CMD stream | torque off ≤ 200 ms, fault bit3, recoverable | phase folder G# |
| Bench actuator ID | step/sweep metrics on real servo (st3215_diag) vs sim k=60/s | sim `position_proportional_gain` re-tuned to match bench (supersedes 0.6) | phase folder G# |
| Air-walk | `real.launch.py device:=/dev/ttyACM0 gait:=true`, robot in cradle | clean trot in air, no faults, temps stable | phase folder G# |

### P4 — `phase-4-*` standing & walking (first stand → untethered walk)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| First stand ladder | cradle → spotter → free (Protocol §3) | stands at stance pose; servo loads/temps within folder limits | phase folder G# |
| Safety layer | fall detection, torque-off paths, thresholds | armed and test-fired before free standing | phase folder G# |
| **Walk regression table** | the sim metric set ON HARDWARE: realized %, yaw drift, lateral, settle | folder's table filled; becomes the spine's hw row | phase folder G# |
| Estimator on hw | /odom_est drift vs tape measure | within folder bar (sim precedent 4–5 % of distance) | phase folder G# |
| Exit | untethered straight 5 m | repeatable — 10 consecutive runs (Protocol §6a) | phase folder G# |

### P5 — `phase-5-*` perception & autonomy (lidar, SLAM/nav2 on robot)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| Lidar in budget | purchase ≤ ₹24,000 (STL-27L primary, LD19/D500 fallback) | driver up, /scan rate per folder bar | phase folder G# |
| SLAM on robot | map of a known room on estimated odom | folder's quality bar (sim precedent: 8×6 room, clean walls) | phase folder G# |
| Compute budget | mission CPU headroom, no robot-side GUIs | action handshakes never time out (failure tree A6) | phase folder G# |
| Speed retune | nav2 keys for hardware (start 0.10 m/s, raise on evidence) | folder's tuning table | phase folder G# |
| Exit | autonomous indoor A→B mission, self-recovering | mission SUCCEEDED by telemetry | phase folder G# |

### P6 — `phase-6-*` RL (3 compute tracks + bypass)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| Track selection | compute reality (cloud GPU / x86+RTX / CPU-only) | flowchart outcome recorded; **no track ⇒ bypass is a PLANNED outcome, not failure** | phase folder G# |
| Env + reward spec | sim env matches bench-ID'd actuators (P3) | spec doc + reproducible training config | phase folder G# |
| Policy vs classical | the P4 walk regression table, policy vs gait | policy ≥ classical on the table, else iterate/bypass | phase folder G# |
| Safe deployment | action scaling staged from 0.25, cradle-first | no safety-layer violations during rollout | phase folder G# |
| Bypass ladder | classical improvement rungs (estimator feedback first) | adopted through ≥ rung 2 if RL is skipped (Protocol §6c) | `05_NO_RL_BYPASS.md` |

### P7 — `phase-7-*` field (courses, runbook, maintenance)
| Theme | What is measured | Bar | Authoritative |
|---|---|---|---|
| Standard course | the folder's acceptance course (the sim obstacle course's hw analog) | completed within folder tolerances | phase folder G# |
| Runbook in use | field sessions executed from checklists alone | two consecutive sessions (Protocol §6d) | phase folder G# |
| Maintenance | inspection/torque/battery schedule | in use, with log entries | phase folder G# |

## 2. The anti-regression spine (must stay green through ALL phases)

| # | Check | Command (crib §) | Pass bar |
|---|---|---|---|
| S1 | barq_control pytest suite | C §SIM/tests | **30 pass** (copyright check may skip) |
| S2 | Firmware codec + build | C §HOST/pio | `pio test -e native` 6/6; `pio run -e teensy41` SUCCESS |
| S3 | Stage-4 stack on firmware logic | `ros2 run barq_hw integration_pty.py` | **9/9**, exit 0 |
| S4 | Sim walk regression | `sim_walk_metric.py --vx 0.15`, fresh spawn, duty 0.6 | **≥ 55 % realized**, yaw within D-019 envelope |
| S5 (P3+) | Drop-in on real Teensy | P3's integration procedure on `/dev/ttyACM0` | P3 gate bar |
| S6 (P4+) | Hardware walk regression table | P4's table run | P4 gate bar |
| S7 (P5+) | Standard mission | A→B on the standard course | mission SUCCEEDED |

**The rule: ANY code change during hardware phases re-runs S1–S4 the same day (plus S5–S7 if the
change touches their layer). A red spine blocks all gate attempts.**

One block, copy-paste (host shell on the Jetson; S1–S4):

```bash
# ── REGRESSION SPINE S1–S4 ──────────────────────────────────────────────────
set -e
# S1: build + unit tests (ephemeral container)
docker run --rm --runtime nvidia --network host --shm-size=8g -v /dev/shm:/dev/shm \
  -v ~/barq_ws:/root/barq_ws barq:dev bash -lc '
    source /opt/ros/humble/setup.bash && cd /root/barq_ws &&
    colcon build --symlink-install 2>&1 | tail -3 &&
    source install/setup.bash &&
    cd src/barq_control && python3 -m pytest test/ -q'
# S2: firmware codec tests + Teensy build (host PlatformIO)
export PATH="$HOME/.local/bin:$PATH"
( cd ~/barq_ws/src/barq_firmware && pio test -e native && pio run -e teensy41 2>&1 | tail -3 )
# S3: full control stack vs real firmware logic, zero hardware (self-isolates on domain 42)
docker run --rm --runtime nvidia --network host --shm-size=8g -v /dev/shm:/dev/shm \
  -v ~/barq_ws:/root/barq_ws barq:dev bash -lc '
    source /opt/ros/humble/setup.bash && source /root/barq_ws/install/setup.bash &&
    ros2 run barq_hw integration_pty.py'
# S4: fresh-spawn sim walk metric (headless sim up, measure, tear down)
docker run --runtime nvidia -d --name spine_sim --network host --shm-size=8g \
  -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws barq:dev bash -lc '
    source /opt/ros/humble/setup.bash && source /root/barq_ws/install/setup.bash &&
    ros2 launch barq_bringup sim.launch.py gait:=true gui:=false'
sleep 45 && docker logs spine_sim --tail 5     # check the LOG, not the exit code
docker exec spine_sim bash -lc '
    source /opt/ros/humble/setup.bash && source /root/barq_ws/install/setup.bash &&
    python3 /root/barq_ws/src/diagnostics/sim_walk_metric.py --vx 0.15 --duration 10'
docker stop spine_sim && docker rm spine_sim
# PASS = "30 passed" · "6 Succeeded"+teensy41 SUCCESS · "9/9" · WALK line ≥55 % realized
```

Record every spine run's S4 WALK line in `docs/05_RESEARCH_LOG.md` if it moved more than noise —
the time series is the project's drift detector.
