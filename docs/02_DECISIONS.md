# BARQ — Decision Log

ADR-style. Newest first. Each decision: context, the call, and why. Referenced from code + changelog.

---
## D-005 — Update robot_params.yaml from the URDF, not CAD
**Date:** 2026-06-09 · **Status:** Accepted
The file shipped with placeholder CAD values (body 0.218x0.108x0.05, coxa/femur/tibia 0.04/0.08/0.11,
symmetric hip offsets). The URDF is now the authoritative geometry, so `robot_params.yaml` was
updated to match it exactly (incl. asymmetric hip offsets). Servo block (IDs/directions/limits)
left untouched — that is a hardware-mapping concern, not geometry. **Why:** single source of truth
must agree with the URDF; downstream IK/gait read these values.

## D-004 — Stage 2A launch loads URDF via `xacro` substitution
**Date:** 2026-06-09 · **Status:** Accepted
`visualize.launch.py` feeds `robot_description` through `Command(['xacro ', urdf_path])` rather than
a plain file read. **Why:** identical behaviour for a plain `.urdf` today, but future-proof if the
description is parameterised into xacro later. `xacro` is already in the image.

## D-003 — `barq_description` installs urdf/meshes/config to `share/`
**Date:** 2026-06-09 · **Status:** Accepted
Added `install(DIRECTORY urdf meshes config DESTINATION share/${PROJECT_NAME})` to the CMakeLists.
**Why:** without it, `package://barq_description/...` cannot resolve after `colcon build`, so RViz
and every downstream node would fail to find the model.

## D-002 — URDF meshes referenced via `package://`
**Date:** 2026-06-09 · **Status:** Accepted
Rewrote all 24 mesh refs from bare `filename="coxa.dae"` to
`filename="package://barq_description/meshes/coxa.dae"`. **Why:** bare filenames resolve relative to
the process CWD and break in RViz/robot_state_publisher; `package://` is the ROS standard.

---
## Decisions inherited from the kickoff doc (recorded for traceability)
## D-001 — Mirroring absorbed at the hardware layer (Option A)
**Status:** Accepted (team)
All joints are commanded in a symmetric/semantic frame. `direction` multipliers in
`robot_params.yaml` absorb physical L/R mirroring (currently coxa only: FR/RR = -1). IK, gait, and
the RL policy never handle mirroring. **Why:** keeps all higher-level math symmetric and simple.

## D-000 — Python everywhere except the hardware boundary
**Status:** Accepted (team)
C++ only for the `ros2_control` hardware interface and the Teensy firmware; everything else Python.
**Why:** development speed; the real-time-critical layer stays compiled.
