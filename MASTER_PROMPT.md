# BARQ MASTER PROMPT — hand this to any assistant or human, first

> **You** (the reader — an LLM assistant of any capability, or a human engineer) are picking up
> BARQ: a 12-servo quadruped robot built by Aryaman Gupta + Krish Agarwal. This document is the
> **complete navigational index** of the project. Its job is to let you find the exact file that
> answers any question **without crawling or re-indexing the workspace**. Everything below is a
> precise pointer with a statement of what lives there and when to open it. Trust this map; it
> was generated from the actual tree at the commit in §8.
>
> Written 2026-06-13 at repo commit `eddb7a3`, branch `stage-2`. If the repo has moved on, §8
> tells you how to refresh your picture in three reads — the map's *paths* stay valid because
> document locations are a stable convention of this project.

---

## 1. What BARQ is, in nine lines

A quadruped: 12× Waveshare ST3215 serial-bus servos (30 kg·cm @12 V) on 4 UART buses via
Waveshare driver boards → Teensy 4.1 (bare-metal 500 Hz superloop, custom binary protocol over
USB) → Jetson Orin Nano (ROS 2 Humble inside Docker image `barq:dev`). Sensors: BNO085 IMU,
INA260 power monitors (owned), STL-27L-class lidar (planned, sim model already exact). Power:
one 4S LiPo through a mandatory 12 V buck for the servo rail (D-021). Software: ros2_control →
exact analytical IK → trot gait ← /cmd_vel, with slam_toolbox + nav2 autonomy and a legged-
odometry estimator; RL locomotion planned as Stage 5. **Current reality: the ENTIRE stack runs
and is validated in simulation (walking, SLAM, autonomous obstacle-course missions) and against
an emulated Teensy; no physical part has been assembled yet — parts are arriving.**

## 2. Prime directives (the project's standing law)

1. **Precedence when sources disagree:** code + config files > dated docs (CHANGELOG/DECISIONS/
   RESEARCH_LOG, newest first) > roadmap > 00_OVERVIEW. Known-stale spots are listed in §4.1.
2. **Docs discipline is non-negotiable** (Aryaman's standing instruction): every change/decision
   updates `docs/` in the same commit — what changed → `03_CHANGELOG.md`; what now stands and
   why → `02_DECISIONS.md` (D-NNN); what an experiment measured and why the change was an
   improvement → `05_RESEARCH_LOG.md`; ambiguities → `04_OPEN_QUESTIONS.md` (Q-NNN).
3. **Git:** commit per milestone on `stage-2`, push after each. Author is ALWAYS
   `Aryaman Gupta <rayman3304@gmail.com>` (set in repo-local AND global git config on the
   Jetson — do not override). SSH to GitHub runs over port 443 (already configured).
4. **Never invent constants.** Unknown values are TBD-table rows naming the measurement that
   fills them. This convention saturates the roadmap; follow it in anything you write.
5. **Verification over confidence:** check log tails not exit codes; verify at the lowest layer
   that can lie; one change at a time, measured before/after. (History of why: research log §3.)
6. **Safety on hardware:** the rules live in `docs/roadmap/00_DOOMSDAY_PROTOCOL.md` §3 and are
   absolutes (deadman never disabled, cradle-first progression, LiPo handling, fingers out).

## 3. How to use this map (anti-crawl directive)

Do **not** glob/grep/explore the workspace to orient yourself — this file + at most three reads
from §5's routing table fully orient you for any task class. Exploration is only justified when
(a) the routing table has no row for your task AND (b) §4's per-file descriptions don't name an
owner. That situation should be rare; if it happens, also fix THIS file afterward (it's a doc
like any other — keep it true).

---

## 4. THE COMPLETE MAP

All paths relative to the repo root `~/barq_ws/src/` (which is also what the Docker container
mounts at `/root/barq_ws/src`). The workspace build artifacts (`~/barq_ws/{build,install,log}`)
and run artifacts (`~/barq_ws/artifacts/` — bags, CSVs) sit OUTSIDE the repo by design.

### 4.1 Living docs — `docs/` (what IS true, maintained continuously)

| File | What it contains | Open it when |
|---|---|---|
| `docs/README.md` | one-page index of the docs system itself | you forget the system |
| `docs/00_OVERVIEW.md` | mission, hardware topology, control-graph architecture, frame conventions (REP-103), verified URDF geometry tables | you need architecture intent. **STALE SPOTS — do not trust:** body mass 0.95 kg (real: robot_params.yaml, 2.448 kg total), tibia limit ±1.57 (real: [−2.2, 0], D-012), "INA226" (owned: INA260, D-021) |
| `docs/01_STATUS.md` | where the project stands NOW: done-list with commit hashes, how-to-run commands, next steps | every session start; after any milestone |
| `docs/02_DECISIONS.md` | ADR log D-000…D-022, newest first: context → the call → why. Code comments cite these IDs | before changing anything load-bearing — check if a D-number already rules it |
| `docs/03_CHANGELOG.md` | dated entries, newest first: WHAT concretely changed per milestone | refreshing your picture after time away (§8) |
| `docs/04_OPEN_QUESTIONS.md` | Q-001…Q-016: pending ambiguities; resolved ones keep their entry with "RESOLVED → D-NNN" | before investigating any oddity — it may be a known, even SOLVED, question |
| `docs/05_RESEARCH_LOG.md` | the publication record: metric series (walk benchmarks, tracking RMS, sweeps), every OVERRIDDEN decision with measured deltas, methodology lessons | writing up results; before re-running any experiment (the baseline numbers live here) |
| `docs/06_PROTOCOL.md` | BARQ binary protocol v1 spec: framing (magic 0xBA51, CRC16-CCITT-FALSE), CMD/STATE/PING-PONG layouts, field scalings, fault bits, golden-vector policy | touching anything that speaks Jetson↔Teensy bytes |
| `docs/HANDOFF.md` | the 1-page quick bootstrap: read order, what is PROVEN (don't re-verify), current frontier, working agreements | fast session resume when you don't need this full map |
| `docs/research/2026-06-11-lidar-selection.md` | the lidar market analysis (A2M12 rejected, STL-27L recommended, full spec tables, Jetson serial prep findings) | lidar purchase or integration work |

**Relationship of the two systems:** `docs/` records what IS; `docs/roadmap/` (below) records
what is PLANNED and exactly how to execute it. A roadmap procedure, once executed, produces
research-log entries / decisions / changelog lines in `docs/`.

### 4.2 The Doomsday Roadmap — `docs/roadmap/` (the forward plan, LLM-optional by design)

Conventions used in every roadmap file: measurable **acceptance gates** `G<phase>.<n>`;
**fallback ladders** A→B→C with explicit switch criteria; **rollback** sections; **TBD-tables**
naming the producing procedure; header `verified against repo @ 4ea53a0`; each file ends with an
"if this entire approach fails" escape hatch.

| File | One-line content statement |
|---|---|
| `roadmap/README.md` | roadmap index: phase table P0–P7 with exit criteria, hard truths incl. power architecture + stale-doc corrections, conventions |
| `roadmap/00_DOOMSDAY_PROTOCOL.md` | the execution operating system: per-session loop, transferable lessons from sim, safety absolutes, stuck-procedure, phase dependency graph, definition of done |
| **P0** `phase-0-environment/01_REBUILD_FROM_ZERO.md` | bare Jetson → flashed, dockered, repo cloned, overlay built, PlatformIO + VNC working; gates reproduce sim walk + 9/9 integration |
| `…/02_BOM_PROCUREMENT.md` | owned-vs-to-buy tables, 12 V buck sizing worksheet, wire/fuse/connector criteria, bench-power and e-stop option ladders, lidar budget row (₹24k ceiling) |
| `…/03_BENCH_SETUP.md` | bench station layout, per-servo hookup chain, LiPo safety zone, parametric test-cradle build, instrument ladder |
| **P1** `phase-1-power-electronics/01_POWER_TREE.md` | the 4S→{Jetson direct, 12 V buck→servo rail, 5 V BEC} tree with sizing math, brownout ladder (14.0/13.8/13.6/13.2 V), battery SOP |
| `…/02_MONITORING_INA260.md` | INA260 placement within 15 A/unit, I2C addressing, register→protocol-field conversion, INA226+shunt fallback design |
| `…/03_TEENSY_AND_BUS_WIRING.md` | which 4 hardware serials and why (I2C collision-aware), half-vs-full-duplex determination procedure, bus↔leg mapping, baud fallback ladder |
| `…/04_BNO085_BRINGUP.md` | I2C/SPI/UART-RVC interface ladder, REP-103 axis-sign verification tables, mount transform TBD |
| **P2** `phase-2-calibration-assembly/01_SERVO_BENCH_CALIBRATION.md` | per-servo: ID assignment (0–11 table), midpoint, sweep, step-response capture; calibration YAML schema → `barq_description/config/servo_calib/` |
| `…/02_ASSEMBLY_SEQUENCE.md` | horn-at-midpoint-at-URDF-zero discipline with jig angles, leg-by-leg order, cable routing, back-drivability gates |
| `…/03_ZEROING_LIMITS_VALIDATION.md` | zero_offset + direction population procedure, the D-012 physical tibia-fold collision check, powered pose checks on the cradle |
| **P3** `phase-3-firmware-integration/01_SERVO_BUS_DRIVER.md` | fills `servo_bus_*` stubs: STS register map (sourced from st3215_diag.py), sync-write + read-strategy ladder with 1 Mbaud timing budget, counts↔mrad conversion, fault bit0 policy |
| `…/02_IMU_POWER_INTEGRATION.md` | fills `imu_read`/`power_read`: SH-2 setup, protocol unit scalings, staleness→bit1, INA260 polling→bit2, the firmware-vs-Jetson split of brownout reactions |
| `…/03_HIL_VALIDATION.md` | the graduation sequence: emulator tests still green → flash → integration_pty.py on real /dev/ttyACM0 → bench step-response → **re-tune sim k to bench** → cradle air-walk → thermal soak |
| **P4** `phase-4-standing-walking/01_FIRST_STAND.md` | torque-ramp staged stand, load-balance metric, oscillation triage, settle-height vs model, backward-tip ladder |
| `…/02_FIRST_STEPS.md` | walking progression 0.02→0.15 m/s with per-level gates, hardware walk metric (tape-truth), tuning decision tree (ORDER matters), surface ladder |
| `…/03_ESTIMATOR_AND_SAFETY.md` | estimator drift protocol on hardware, **the stop ladder** (critical finding: killing the gait node does NOT trip the deadman), required code additions P4-CODE-1…6 (incl. /barq/diag + /imu/data bridge in barq_hw), fall detection, drills |
| **P5** `phase-5-perception-autonomy/01_LIDAR_PURCHASE_AND_INTEGRATION.md` | purchase gate under ₹24k, driver bringup, udev rules (barq_teensy/barq_lidar), the −4.5° mount-wedge fabrication + level verification, self-hit filter |
| `…/02_ONROBOT_SLAM_NAV.md` | robot.launch.py composition spec, hardware deltas for slam/nav2 yamls (sim-only odom_topic/use_sim_time flagged), first-map protocol, mission protocol with robust action client |
| `…/03_COMPUTE_BUDGET.md` | Orin load measurement procedure, nvpmodel/thermal, the 5-rung mitigation ladder (shed GUIs → priorities → rates → offload viz → wifi-DDS last resort) |
| **P6** `phase-6-rl/01_STRATEGY_AND_COMPUTE_TRACKS.md` | what the policy is/is not; Track A cloud GPU / B local RTX / C Mac-CPU fully specified; selection flowchart; cost-estimation method |
| `…/02_ENV_AND_REWARD_SPEC.md` | code-ready: URDF→MJCF + parity script, 45-dim obs table with indices, action = offsets around the TRIMMED stance, reward table with formulas/weights, DR table, curriculum, terminations |
| `…/03_TRAINING_RECIPES.md` | PPO hyperparameter tables per track, failure-signature table (which knob first), run-management discipline, in-sim eval gates incl. beat-the-gait bar |
| `…/04_SIM2REAL_AND_DEPLOYMENT.md` | pre-transfer checklist (bench-ID'd actuator, measured latency), ONNX export + parity test, `rl_policy_node` + shared `rl_obs.py` spec, safety wrapper, hardware deployment ladder, obs-gap iteration loop |
| `…/05_NO_RL_BYPASS.md` | the 6-rung classical ladder (body-vel feedback → yaw feedback → pitch reflex → cmd shaping → stumble reflex → MPC-lite honesty note); rungs 1–2 worth doing even if RL succeeds |
| **P7** `phase-7-field/01_ACCEPTANCE_COURSES.md` | 5 graduated physical courses incl. the sim obstacle-course replica at true SDF coordinates, per-course regression tables bound to sim baselines |
| `…/02_OPERATIONS_RUNBOOK.md` | phone-usable pre-run/run/post-run checklists, crash-response tree, maintenance clocks (5 h/20 h), spares, transport pose |
| **Appendices** `appendices/A_FAILURE_TREES.md` | symptom → ordered checks → fix, for EVERY failure class this project has hit plus predictable hardware ones |
| `…/B_ACCEPTANCE_GATES.md` | master gate themes per phase + **the regression spine** (the test set that must stay green through everything, with the one command block that runs it) |
| `…/C_COMMAND_CRIB.md` | every command copy-paste ready: host/docker/sim/real/diagnostics/slam-nav/debug groups |
| `…/D_RISK_REGISTER.md` | 20 risks with likelihood/impact/early-signal/mitigation/contingency; top-3 callout |
| `…/E_PARAMETER_REGISTRY.md` | EVERY tunable: current value, owning file, consumer, re-derivation procedure, D-number provenance, sim/hw scope. **This is where you look up any number before grepping code** |

### 4.3 Code map — packages and their load-bearing files

| Path | What it is |
|---|---|
| `barq_description/urdf/barq.urdf.xacro` | THE robot model: exact masses/inertias, joint limits (effort 2.94, velocity 4.71), ros2_control block with `mode:=mock\|gazebo\|real` + `device` + `foot_mu` args, stance init values, foot contact spheres, lidar (laser link, −4.5° wedge) + IMU sensors, gazebo plugin block |
| `barq_description/config/robot_params.yaml` | single source of truth: measured masses, actuator specs, EXACT leg geometry constants, hip offsets, servo ID/direction/zero_offset table |
| `barq_description/meshes/` | visual DAE meshes (coxa/femur/tibia/body/feet) |
| `barq_control/barq_control/leg_kinematics.py` | `fk_exact`/`ik_exact` — URDF-true analytical kinematics (D-014); legacy `fk_leg/ik_leg` marked not-for-control |
| `barq_control/barq_control/gait.py` | pure-function trot generator: stance sweep + smoothstep swing (D-019), all gait math |
| `barq_control/barq_control/gait_planner_node.py` | ROS node: /cmd_vel → foot targets @50 Hz; params period/duty/step_height/stand_height/rear_raise/forward_sign; 1 s cmd deadman |
| `barq_control/barq_control/ik_node.py` | foot targets → 12 joint positions → `/joint_group_position_controller/commands`; ankle clamps |
| `barq_control/barq_control/state_estimator_node.py` | stance-diagonal legged odometry + IMU yaw → /odom_est (+TF when `odom_source:=estimated`) |
| `barq_control/barq_control/odom_tf_node.py` | ground-truth odom→TF with forced frame names (sim only) |
| `barq_control/barq_control/barq_protocol.py` | Python protocol codec (golden-vector-pinned twin of the C++) |
| `barq_control/test/` | the unit suite (30 pass + 1 skip): exact-kinematics vs URDF chain, gait properties, IK, estimator, protocol golden vectors, lint |
| `barq_bringup/launch/sim.launch.py` | Gazebo sim bringup; args: world_file, gui, gait, slam, nav, odom_source, foot_mu, gait_duty, gait_period; prepends the patched-plugin path |
| `barq_bringup/launch/real.launch.py` | THE hardware launch (identical for emulator PTY and real Teensy: `device:=`) |
| `barq_bringup/launch/control.launch.py`, `visualize.launch.py` | kinematic-only RViz stacks (Stage 2A/2B era; still useful for posture checks) |
| `barq_bringup/config/ros2_controllers.yaml` | controller_manager 100 Hz, the 12-joint position controller, **and the sim servo stiffness `position_proportional_gain: 0.6`** |
| `barq_bringup/config/barq_slam.yaml`, `barq_nav2.yaml` | SLAM + nav2 tuning (sim-tuned; hardware deltas specified in roadmap P5-02) |
| `barq_bringup/rviz/barq_slam.rviz` | the mission RViz profile (run on the Mac, not the robot) |
| `barq_sim/worlds/barq_world.sdf`, `barq_course.sdf` | 8×6 walled room + pillars; 10×8 obstacle course (doorway/slalom/boxes) |
| `barq_hw/src/barq_system.cpp` (+ `include/barq_hw/barq_system.hpp`) | the C++ ros2_control hardware interface: CMD out @100 Hz, STATE in, live-STATE activation, stale-link ERROR |
| `barq_hw/src/teensy_emulator.cpp` | the real firmware LoopCore served on a PTY — zero-hardware integration target |
| `barq_hw/test/integration_pty.py` | the 9-check Stage-4 contract test (also `ros2 run barq_hw integration_pty.py`); works on emulator PTY today and /dev/ttyACM0 later |
| `barq_firmware/` (COLCON_IGNOREd; PlatformIO) | `src/loop_core.{h,cpp}` = ALL superloop logic (shared verbatim with the emulator — keep it Arduino-free); `src/main.cpp` = thin Teensy shim; `src/protocol.{h,cpp}` = C++ codec; `test/test_protocol/` = 6 Unity tests; stubs to fill at P3: `servo_bus_*`, `imu_read`, `power_read` |
| `diagnostics/st3215_diag.py` (+ README) | the servo BENCH tool: scan/ping/set-id/calibrate-mid/move/sweep/limits — and the source of the STS register map |
| `diagnostics/sim_actuation_probe.py` | sim step-response + tracking probe (the metrics that match the bench) |
| `diagnostics/sim_walk_metric.py` | the WALK regression line (fwd/lat/yaw/realized-%) |
| `diagnostics/analyze_track_bag.py` | offline tracking-error analysis from a ros2 bag (immune to live-sampling starvation) |
| `external/gz_ros2_control/` | vendored+patched sim plugin (PROVENANCE.md + BARQ.patch). **Fresh clones MUST build it once** or the sim silently reverts to the soft servo (canary: `position_proportional_gain` reads 0.1) |
| `Dockerfile` | the barq:dev image recipe (dustynv base + gz slim set + slam/nav layers, with the OpenCV-conflict workarounds) |
| `MASTER_PROMPT.md` | this file |

---

## 5. TASK ROUTING TABLE — read exactly this, in this order, nothing else

| Your task | Read (in order) |
|---|---|
| **Resume development, any kind** | `docs/01_STATUS.md` → `docs/HANDOFF.md` → tail of `docs/03_CHANGELOG.md` |
| **Understand a past decision / "why is X like this?"** | `docs/02_DECISIONS.md` (search the D-number cited in code) → if it was an override, `docs/05_RESEARCH_LOG.md` for the measured why |
| **Something looks broken** | `docs/roadmap/appendices/A_FAILURE_TREES.md` (find the symptom) → `docs/04_OPEN_QUESTIONS.md` (is it a known Q?) |
| **Look up any number/tunable** | `docs/roadmap/appendices/E_PARAMETER_REGISTRY.md` → only then the owning file it names |
| **Run anything (commands)** | `docs/roadmap/appendices/C_COMMAND_CRIB.md` |
| **Start hardware work, phase N** | `docs/roadmap/00_DOOMSDAY_PROTOCOL.md` → `docs/roadmap/phase-N-*/` files in numeric order → your phase's rows in `appendices/B_ACCEPTANCE_GATES.md` |
| **Parts arrived, where do I begin?** | `docs/roadmap/README.md` phase table → P0 gates (is the env alive?) → P1 |
| **Servo bench work** | `diagnostics/README.md` → `docs/roadmap/phase-2-calibration-assembly/01_…` |
| **Firmware work** | `docs/06_PROTOCOL.md` → `barq_firmware/README.md` → `docs/roadmap/phase-3-firmware-integration/` |
| **Gait/walking tuning** | `docs/05_RESEARCH_LOG.md` §2f (the baselines) → `docs/roadmap/phase-4-standing-walking/02_FIRST_STEPS.md` (the decision tree) → Q-013/Q-016 in `docs/04_OPEN_QUESTIONS.md` |
| **Sim fidelity / actuation questions** | D-018 + D-019 in `docs/02_DECISIONS.md` → `external/gz_ros2_control/PROVENANCE.md` |
| **Lidar/SLAM/nav on robot** | `docs/research/2026-06-11-lidar-selection.md` → `docs/roadmap/phase-5-perception-autonomy/` |
| **RL, any aspect** | `docs/roadmap/phase-6-rl/01…05` in order (02 is the env contract; 05 is the no-GPU bypass) |
| **Field session / demo day** | `docs/roadmap/phase-7-field/02_OPERATIONS_RUNBOOK.md` (it is self-sufficient) |
| **Writing/updating docs** | `docs/README.md` (the system) → the file ownership table in §4.1 above |
| **Modifying protocol bytes** | STOP: `docs/06_PROTOCOL.md` golden-vector policy — regenerate vectors in BOTH languages in the same commit, never hand-edit |
| **Adding a dependency / changing the image** | `Dockerfile` comments + `docs/roadmap/phase-0-environment/01_REBUILD_FROM_ZERO.md` landmine table |

## 6. Convenience facts card (copies — authoritative source cited per line)

- Total mass ≈2.448 kg; body 1420 g INCLUDING the 512 g 4S battery (robot_params.yaml; D-021).
- Joint limits: hips ±0.785, knees ±1.57, ankles [−2.2, 0]; effort 2.94 N·m; velocity 4.71 rad/s
  (URDF + registry; D-012/D-018). Tibia full-fold reach floor: 0.1079 m in-plane (D-019).
- Stance: stand_height 0.13, rear_raise 0.02 → body pitch ≈ −4.5° nose-down BY DESIGN (D-016).
- Rates: gait 50 Hz → controllers/CMD/STATE 100 Hz → firmware 500 Hz; deadman 200 ms (bit3);
  gait cmd_vel deadman 1 s (loop_core.h, gait_planner_node.py).
- Protocol: magic 0xBA51, CRC16-CCITT-FALSE, CMD int16 mrad ×12 in servo-ID order
  (FL,FR,RL,RR × hip,knee,ankle = IDs 0–11), STATE 98 B (06_PROTOCOL.md).
- Sim baselines to not regress: realized ≈60 % @ vx 0.15 (duty 0.6), tracking 17.8 mrad mean RMS,
  estimator drift 4–5 %, 30+1 pytest green, 6/6 pio native, 9/9 integration (research log; B appendix).
- Container rune essentials: `--network host -v /dev/shm:/dev/shm -v ~/barq_ws:/root/barq_ws`,
  one DDS stack at a time (or ROS_DOMAIN_ID), `timeout -k 2` on every ros2 CLI (C appendix).

## 7. Known traps (each documented at the cited location — listed here so you don't rediscover them)

- 00_OVERVIEW stale spots (§4.1 row). · Sim silently soft if the vendored plugin isn't built
  (canary param 0.1 — failure tree A). · Killing the gait node does NOT torque-off (stop ladder,
  P4-03). · nav2/slam yamls carry sim-only settings (P5-02 deltas). · Live rclpy metrics lie
  under load — use bags (research log 2f lesson 3). · ros2 CLI swallows SIGTERM in containers
  (use `timeout -k 2`). · Realized speed ≈60 % of command is the EXPLAINED baseline, not a bug
  (Q-013 RESOLVED). · Yaw-vs-duty trade is open (Q-016): duty 0.6 fast+veer / 0.55 straight.
- The 4S pack must NEVER feed servos/driver boards directly — 12 V buck only (D-021, P1-01).

## 8. State snapshot + how to refresh it

**As of `eddb7a3` (2026-06-13):** Stages 2 (sim) and 4-interface are DONE and validated; Stage 3
firmware is protocol-complete with hardware stubs pending parts; the sim is actuation-calibrated
to the servo spec; autonomy (SLAM+nav2+estimator) is course-proven in sim; the Doomsday Roadmap
(33 docs) is the forward plan; power architecture is decided (D-021). No hardware assembled yet.
Open: Q-014 (CoM, measure battery-installed), Q-016 (yaw-vs-duty), lidar purchase (P5-01 gate).

**To refresh after time away (exactly three reads):** `git log --oneline -15` → head of
`docs/03_CHANGELOG.md` → `docs/01_STATUS.md`. If those mention files this map doesn't know,
update §4 of this file in the same session.

## 9. If you are an LLM assistant, additionally

- This machine (the Jetson) may hold Claude-specific memory files for past assistants; they are
  machine-local conveniences. THIS file is the portable, repo-versioned equivalent — prefer it.
- Mirror the user's working agreements (§2) without being asked: update docs with every change,
  commit per milestone as Aryaman, push, keep the research log publication-grade.
- When the user says "continue" with no context: §8's three reads, then propose the next
  unchecked gate from `appendices/B_ACCEPTANCE_GATES.md` or the next item in `01_STATUS.md`.
- Token-constrained? The minimum viable orientation is: §1, §2, §5 (your task's row), §7. That
  is ~2 pages and sufficient for safe work.
