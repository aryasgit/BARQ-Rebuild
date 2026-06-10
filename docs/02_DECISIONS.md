# BARQ — Decision Log

ADR-style. Newest first. Each decision: context, the call, and why. Referenced from code + changelog.

---
## D-011 — cmd_vel is robot-centric; forward = head-first (mapped at the gait layer)
**Date:** 2026-06-10 · **Status:** Accepted
`/cmd_vel +x` means "walk head-first" (robot front = the body's -X end per the mesh; the URDF leg
labels disagree — Q-012). The mapping lives in ONE place: `gait_planner_node` negates linear x,y into
body axes (yaw unchanged). URDF, IK, and the pure gait generator stay in clean body-frame math.
Crouch defaults: stand_height 0.16, step_height 0.012 (stand-step must stay >= ~0.147 m or the swing
apex exceeds the -1.57 tibia limit). **Why:** smallest possible surface for the direction convention;
trivially flippable at 2E when ground contact makes direction unambiguous.

## D-010 — Simulator: Gazebo for 2E, MuJoCo for RL (both)
**Date:** 2026-06-10 · **Status:** Accepted (resolves Q-002; re-recorded after the f4cd735 revert)
Gazebo (`gz_ros2_control`) for the Stage 2E control-stack sim — our controllers/launch drop in as-is.
MuJoCo/Isaac for RL training at Stage 5. Different jobs; they coexist. 2E adds Gazebo to the Dockerfile.

## D-009 — IK knee-bend branch = -1 (legs fold forward)
**Date:** 2026-06-10 · **Status:** Accepted (resolves Q-010)
The analytical IK has two mirrored elbow branches reaching the same foot point. Visual check on the
rendered robot showed +1 folds the legs backward (tibia closing rearward); BARQ's physical config is
the forward fold. Default is now **-1** in `leg_kinematics.ik_leg` and the `ik_node` `knee_bend` param
(±1 still selectable). Corroborated by the servo tibia range **[-1.571, 0]** in robot_params — the +1
branch commanded ankle angles (+1.52) the hardware cannot reach. Foot placement unchanged (FK round-trip
tested, `test_forward_fold_branch`). Travel-direction/±X conventions deliberately unchanged (deferred).

---
## D-008 — Gait: open-loop trot, /cmd_vel -> /foot_targets
**Date:** 2026-06-10 · **Status:** Accepted
`gait.py` generates a trot (diagonal pairs FL+RR / FR+RL, 50% duty): stance foot sweeps back to push
the body, swing foot lifts on a sine arc and returns; step size = body velocity x stance time (incl. a
yaw term r x w). `gait_planner_node` maps `/cmd_vel` (Twist) -> `/foot_targets`; the existing IK node
closes the loop. Open-loop for now (no `/robot_state` feedback) — matches the doc's 2D; state estimation
is deferred until there's an IMU (sim/hardware). Defaults `stand_height=0.18`, `step_height=0.03` chosen
so swing stays within the +/-1.57 ankle limit. **Why:** trot is the simplest statically-reasonable gait;
reusing the `/foot_targets` contract keeps gait and IK decoupled and testable. Tuning continues at 2E
against physics; tibia range (Q-001) may change the foot-trajectory envelope.

---
## D-007 — IK: idealized 3-DOF analytical model, body-frame foot targets
**Date:** 2026-06-10 · **Status:** Accepted
`leg_kinematics.py` uses the idealized leg (clean link lengths L1/L2/L3 from robot_params; coxa about X,
femur/tibia about Y), not the URDF's exact mesh-frame offsets. IK is closed-form, validated by FK<->IK
round-trip to 1e-9. `ik_node` consumes `/foot_targets` (12 foot xyz, body frame, FL/FR/RL/RR) and emits
12 joint positions to the position controller @ 50 Hz. Per-leg L/R is handled by the target + the leg's
`side` sign — no separate mirror multiplier in sim; the servo-direction multipliers in robot_params
apply only at the real-hardware layer (see [[D-001]]). `knee_bend=+1` branch (Q-010); outputs clamped to
joint limits. **Why:** the project standardised on clean link lengths; closed-form IK is fast and exact
for that model; the sub-cm URDF offsets are negligible for viz/gait and can be calibrated later.

---
## D-006 — ros2_control: xacro `mode` arg + mock_components + JointGroupPositionController
**Date:** 2026-06-10 · **Status:** Accepted
Description is now `barq.urdf.xacro` with a `mode` arg (mock|gazebo|real); only the `<hardware>` plugin
differs per mode (`mock_components/GenericSystem` now). Command path is a single
`position_controllers/JointGroupPositionController` over all 12 joints (Float64MultiArray on
`/joint_group_position_controller/commands`) — the topic IK (2C) and gait (2D) will publish to.
**Why:** matches the "Mock->Sim->Real, only the hardware interface changes" principle; one grouped
position command is the simplest fit for IK/gait output. Macro params are `lower`/`upper`, never
`min`/`max` (those shadow xacro builtins and break the launch — see CHANGELOG 2026-06-10).

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
