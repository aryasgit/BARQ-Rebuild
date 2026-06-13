# BARQ Roadmap — the Doomsday Guide

> **Purpose:** a complete, self-contained execution plan from "sim works" (where we are) to a
> fully functioning quadruped — calibration, assembly, firmware, control stack, perception,
> autonomy, RL — written so it can be executed **without any LLM assistance**. Every procedure
> has measurable acceptance gates and fallback ladders (if A fails → B → C). If you are reading
> this cold: start with `00_DOOMSDAY_PROTOCOL.md`, then go phase by phase.

## Who/what this assumes
A competent engineer (Aryaman/Krish-level), the BARQ-Rebuild repo at or after commit `0e5ddaf`,
and the parts listed in `phase-0-environment/02_BOM_PROCUREMENT.md`. No other context needed —
but the main `docs/` system (00–06, HANDOFF) is the project's living truth; this roadmap is the
**forward plan**. Where they disagree, the dated docs + code win.

## Phase map (and how it relates to the Stage numbering in docs/)
| Roadmap phase | Covers | Stage label | Gate to exit |
|---|---|---|---|
| **P0** environment | rebuild dev env from bare metal; BOM; bench | — (meta) | sim walk + `integration_pty.py` 9/9 reproduced |
| **P1** power & electronics | 4S power tree (12 V buck!), Teensy, 4 servo buses, BNO085, INA260 | Stage 3 (hw half) | every board talks on the bench |
| **P2** calibration & assembly | per-servo bench calib, IDs, assembly at zero pose, limits, D-012 fold check | Stage 3 | robot assembled, all 12 calibrated, safe limits verified |
| **P3** firmware integration | fill `servo_bus_* / imu_read / power_read` stubs; HIL vs real Teensy | Stage 3 done | `integration_pty.py` passes on `/dev/ttyACM0`; air-walk on cradle |
| **P4** standing & walking | first stand, first steps, gait transfer & tuning, estimator, safety | Stage 4 done | untethered walk, straight 5 m, repeatable |
| **P5** perception & autonomy | lidar purchase+integration, SLAM/nav2 on robot, compute budget | Stage 4.5 | autonomous A→B mission indoors |
| **P6** RL | strategy (3 compute tracks), env+reward spec, training, sim2real, deploy; no-RL bypass | Stage 5 | policy ≥ classical gait on the regression table, or bypass ladder adopted |
| **P7** field | acceptance courses, operations runbook, maintenance | — | course complete + runbook in use |
| **appendices/** | failure trees, master gate table, command crib, risk register, parameter registry | — | reference |

## Read order when starting a phase
1. `00_DOOMSDAY_PROTOCOL.md` (once — the rules of execution)
2. The phase folder's files in numeric order
3. `appendices/B_ACCEPTANCE_GATES.md` row(s) for the phase — know the bar before you start
4. `appendices/A_FAILURE_TREES.md` — skim the symptoms relevant to the phase

## Hard truths the roadmap is built on (with corrections to older docs)
- **Power (team decision, 2026-06-12):** one **4S LiPo, GENX Premium 5200 mAh, 512 g**; full
  charge ≈ 17.1 V (VERIFY chemistry — LiPo 4.20 V/cell vs LiHV 4.35 V/cell — before first
  charge), operational floor **13.6 V** (3.4 V/cell). The 512 g is ALREADY included in the
  1420 g body mass and the pending CoM measurement (Q-014: measure with battery installed).
  Consequence: 4S > the 12.6 V max of the ST3215 servos and Waveshare driver boards →
  **a high-current 12 V buck for the servo rail is mandatory** (P1-01). The Jetson Orin Nano
  (9–20 V input) takes 4S directly, fused.
- **Power monitor: INA260s are owned** (integrated 2 mΩ shunt, ±15 A continuous ceiling per
  unit), not the INA226 named in older docs/protocol comments. The protocol's vbus/current
  fields are chip-agnostic; P1-02 designs monitoring within the 15 A/unit limit (INA226 +
  external shunt is the documented fallback if any rail must carry more through one monitor).
- **IMU: BNO085 owned.** Lidar: NOT purchased; budget ceiling **₹24,000**; STL-27L is primary
  (~₹13k, 45 g, 25 m, matches the sim lidar already built); LD19/D500 fallback (P5-01).
- **Stale spots in 00_OVERVIEW.md** (kept for history): body mass 0.95 kg and tibia limit
  ±1.57 are superseded — trust `barq_description/config/robot_params.yaml` (masses, 2.448 kg
  total) and D-012 (tibia [−2.2, 0]).
- Sim is calibrated to the servo spec (D-018: 2.94 N·m, 4.71 rad/s, k=60/s) and gets
  **re-calibrated to the bench** in P3-03 — the same step metrics exist on both sides.

## RL compute reality (asked 2026-06-12)
Access to x86+RTX is **uncertain**. P6 therefore specifies THREE complete tracks — (A) rented
cloud GPU, (B) local x86+RTX Isaac Lab, (C) Mac/Jetson-only CPU MuJoCo — with a selection
flowchart, plus `05_NO_RL_BYPASS.md`: a classical-control improvement ladder that reaches a
deployable robot even if RL never happens.

## Conventions used in every roadmap file
- **Gate `G<phase>.<n>`** — a measurable pass/fail bar. You do not proceed past a failed gate;
  you take that file's fallback ladder.
- **Fallback ladder** — Plan A → B → C with explicit switch criteria ("switch after 2 failed
  attempts or 2 hours, whichever first").
- **TBD-table** — values that must be MEASURED, never guessed (e.g., real stall current, buck
  ripple). Each TBD row names the procedure that produces it. Filling a TBD = a research-log
  entry (`docs/05_RESEARCH_LOG.md`, standing practice).
- File headers carry `verified against repo @ <commit>`. If the repo moved far, re-check the
  file's premises against `docs/03_CHANGELOG.md` before executing.
