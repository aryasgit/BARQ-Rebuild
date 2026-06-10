# BARQ — Open Questions

Pending ambiguities & decisions. When resolved, move the outcome to `02_DECISIONS.md`.

---
## Q-001 — Tibia (ankle) joint limit conflict
URDF + kickoff doc say tibia range is **+/-1.57 rad**. But `robot_params.yaml` servo config says
**[-1.571, 0.0]** (bends one direction only). These disagree. Which is the true mechanical limit?
- If servo is right -> tighten URDF `<limit upper>` to 0.0.
- If URDF is right -> relax the servo entry.
*Impact:* IK joint clamping, gait reachable workspace, safety limits. **-> asked 2026-06-09.**
**Update 2026-06-10:** strong evidence for the servo config — the visually-correct leg fold (D-009,
knee_bend=-1) keeps the tibia in [-1.57, 0] everywhere (stance -1.52; gait -1.03..-1.34). Pending only
a mechanical confirmation; then tighten the URDF upper limit to 0.

## Q-002 — Physics simulator — RESOLVED 2026-06-10 -> D-010
Both: **Gazebo for Stage 2E** (gz_ros2_control, drop-in for our stack; add to Dockerfile) and
**MuJoCo/Isaac for RL** at Stage 5. Decided by Aryaman.

## Q-003 — Git commit/push policy  · BLOCKING for committing anything
URDF + meshes are untracked; Stage 2A changes are uncommitted. Remote: `aryasgit/BARQ-Rebuild`.
How autonomous should commits/pushes be? **-> asked 2026-06-09.**

---
## Non-blocking notes (track, resolve opportunistically)
## Q-010 — IK knee-bend direction — RESOLVED 2026-06-10 -> D-009
Visual check showed +1 folded the legs backward; default flipped to `knee_bend=-1` (forward fold).
Foot positions unchanged; tibia now stays within the servo's [-1.571, 0] range.

## Q-012 — Frame/label story: robot front vs +X vs URDF leg names · partially handled, cleanup deferred
Working model (per Aryaman's mesh reading): the robot's physical FRONT/head is the body's **-X** end —
where the URDF joints *named* RL/RR sit; the FL/FR-named joints are at the tail (+X). As of D-011 the
gait maps cmd_vel so **+x = head-first (toward -X)**; direction now looks right in RViz.
Remaining cleanup (deferred): URDF leg labels don't match physical quadrants. A full rename touches
URDF + ros2_control + controllers.yaml + joint_conventions + servo map + code, and likely a leg-mesh
swap (mid<->mid_rev, foot<->foot_rev). Decide at 2E (ground contact) or against real hardware/CAD.

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
