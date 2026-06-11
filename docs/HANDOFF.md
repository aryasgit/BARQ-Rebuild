# BARQ — Session Handoff Guide

> For a NEW session (human or AI) joining this project cold. This file contains **pointers, not
> details** — follow the read order below and you will know where we are, what is proven, and how
> to work here without re-deriving anything.

## Read in this order
1. **`docs/01_STATUS.md`** — where we are right now, what's done, what's next, how to run things.
2. **`docs/00_OVERVIEW.md`** — the durable reference: mission, hardware, architecture, geometry,
   staged plan (2A→5).
3. **`docs/02_DECISIONS.md`** (newest-first) — every standing decision with rationale. Treat these
   as binding unless the user overrides; note which decisions supersede earlier ones.
4. **`docs/04_OPEN_QUESTIONS.md`** — what is genuinely unresolved (and what was resolved, kept for
   trace). Don't re-ask answered questions.
5. **`docs/05_RESEARCH_LOG.md`** — the iteration/metrics story (overridden decisions and measured
   improvements). **Standing practice: append to it after every experiment or override.**
6. **`docs/03_CHANGELOG.md`** — dated detail of every change; consult as needed, don't read whole.

## Environment facts (or read the memory files)
Claude's memory at `~/.claude/projects/-home-barq/memory/` holds the operational knowledge:
`barq-dev-topology` (tools run ON the Jetson; the Mac is the SSH client), `barq-remote-viz`
(headless display + VNC: fix_display.sh / setup_vnc.sh / run_barq_gui.sh, view at
`vnc://barq.local:5900`), `barq-file-channel` (Mac→Jetson rsync helpers), `barq-git-workflow`
(author = Aryaman Gupta, branch `stage-2`, push per milestone, SSH over 443).

Hard-won rules you must not relearn the hard way:
- **One ROS stack at a time** — host-network containers share one DDS graph (cross-talk).
- **Cross-container ROS needs `-v /dev/shm:/dev/shm` on EVERY container** — FastDDS same-host
  transport is shared memory; without it, discovery works but data silently never arrives.
- Everything ROS runs inside the `barq:dev` Docker image; host has no ROS.
- Gazebo GUI on this Jetson needs `--render-engine-gui ogre` + `IGN_GAZEBO_RESOURCE_PATH`
  (already wired into `sim.launch.py`).
- RViz "treadmill" perception of walking direction is unreliable; judge direction in physics.
- Don't trust `gnome-screenshot` for GL windows; verify via VNC or `ign model -m barq --pose`.

## Code entry points (workspace `~/barq_ws/src`)
- `barq_description/` — URDF (`urdf/barq.urdf.xacro`, xacro `mode:=mock|gazebo`), measured
  masses/limits, `config/robot_params.yaml` (single source of truth incl. exact leg geometry).
- `barq_control/barq_control/` — `leg_kinematics.py` (USE `fk_exact`/`ik_exact`; legacy funcs are
  not for control), `gait.py`, `ik_node.py`, `gait_planner_node.py`.
- `barq_control/test/` — 20 tests; `test_exact_kinematics.py` pins the model to the raw URDF chain.
  Run: `cd src/barq_control && python3 -m pytest test/` (inside the container).
- `barq_bringup/launch/` — `visualize` (RViz sliders), `control` (mock loop, `gait:=true`),
  `sim` (Gazebo physics, `gui:=true gait:=true`).
- `barq_sim/worlds/barq_world.sdf` — offline flat-ground world.
- Host helpers: `~/walk_demo.sh [secs] [vx]` (replay the sim walk), `~/run_barq_gui.sh`.

## What is PROVEN (don't re-verify unless something changed)
Stage 2 complete end-to-end: RViz viz → mock ros2_control → exact analytical IK (1e-12 vs URDF) →
trot gait → **Gazebo physics: stands (settle-height error 0.2 mm), walks forward (+X, D-015),
straight, level, with measured masses (2.448 kg) and real torque caps (2.94 N·m), nose-down
stance trim (D-016)**. All pushed to `origin/stage-2` (github.com/aryasgit/BARQ-Rebuild).

## Current frontier
- **Next stage: 3 — Teensy firmware** (BNO085 + INA226 in hand; PlatformIO on the Mac).
- Open: Q-013 (gait realizes ~half of commanded speed; tuning levers listed), Q-014 (exact CoM
  coordinates pending from Aryaman → base_link inertial origin), D-012 follow-up (check physical
  link collision at tibia −2.2 before driving real servos), Q-009 (RT scheduling, Stage 4).

## Working agreements with Aryaman
Maintain the `docs/` system after EVERY change/decision (standing instruction). Commit per
milestone on `stage-2`, push after each, author Aryaman Gupta. Show results live over VNC; verify
in physics before claiming success; surface trade-offs and ask only decision-grade questions.
