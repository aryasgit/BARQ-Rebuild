# BARQ — Open Questions

Pending ambiguities & decisions. When resolved, move the outcome to `02_DECISIONS.md`.

---
## Q-001 — Tibia (ankle) joint limit conflict  · BLOCKING for IK (2C)
URDF + kickoff doc say tibia range is **+/-1.57 rad**. But `robot_params.yaml` servo config says
**[-1.571, 0.0]** (bends one direction only). These disagree. Which is the true mechanical limit?
- If servo is right -> tighten URDF `<limit upper>` to 0.0.
- If URDF is right -> relax the servo entry.
*Impact:* IK joint clamping, gait reachable workspace, safety limits. **-> asked 2026-06-09.**

## Q-002 — Physics simulator: Gazebo vs MuJoCo  · BLOCKING for 2E + Dockerfile
Kickoff doc Stage 2E says **Gazebo Harmonic**. But the Dockerfile installs **MuJoCo** (pip) and has
**no Gazebo**; README says "MuJoCo/Gazebo".
- Gazebo: matches the "Mock->Sim->Real, same ros2_control interface" principle (gz_ros2_control);
  must be added to the Dockerfile.
- MuJoCo: already installed, fast, RL-friendly (aligns with Stage 5); needs a custom bridge to the
  ros2_control / joint interface.
*Impact:* `barq_sim` design, Dockerfile, Stage 2E. **-> asked 2026-06-09.**

## Q-003 — Git commit/push policy  · BLOCKING for committing anything
URDF + meshes are untracked; Stage 2A changes are uncommitted. Remote: `aryasgit/BARQ-Rebuild`.
How autonomous should commits/pushes be? **-> asked 2026-06-09.**

---
## Non-blocking notes (track, resolve opportunistically)
## Q-009 — Real-time scheduling under Docker (defer to Stage 3/4)
ros2_control_node warns "Could not enable FIFO RT scheduling policy (Operation not permitted)" in the
container. Harmless for mock/sim. For real hardware: add RT privileges to the container
(`--cap-add=SYS_NICE --ulimit rtprio=99`, maybe a PREEMPT_RT-aware setup) so the control loop can hit
its rate reliably.

## Q-004 — Coxa visual orientation front vs rear  (likely OK)
Coxa visual `rpy` differs front/rear on the same side (FL roll=pi vs RL roll=0; FR roll=0 vs RR roll=pi).
In the 2A RViz render the stance looks correct and symmetric — no obvious mirror error. Low priority;
give it a final eyeball by orbiting the camera in RViz, then close this.

## Q-005 — Joint declaration order
URDF declares legs **FL, RL, FR, RR**; `joint_conventions.yaml` + servo IDs use **FL, FR, RL, RR**.
Name-based lookup tolerates this, but anything that zips raw joint arrays by index could break.
Keep consistent; prefer name-keyed access everywhere.

## Q-006 — Duplicate Dockerfile
`barq_ws/Dockerfile` and `barq_ws/src/Dockerfile` are byte-identical; only the `src/` one is tracked.
Consolidate to one canonical location.

## Q-008 — Root link inertia (KDL warning)
robot_state_publisher warns that root link `base_link` has an `<inertial>` block and KDL doesn't
support inertia on the root link. Harmless for RViz. For Gazebo (2E): add a massless dummy root link
(e.g. `base_footprint` -> fixed joint -> `base_link`) or relocate the body inertia. Track until 2E.

## Q-007 — Package metadata placeholders
All `package.xml` files have `maintainer root@todo.todo`, `license TODO`, `description TODO`.
Fill in before any public release / proper build hygiene.
