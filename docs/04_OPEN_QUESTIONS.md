# BARQ — Open Questions

Pending ambiguities & decisions. When resolved, move the outcome to `02_DECISIONS.md`.

---
## Q-001 — Tibia joint limit — RESOLVED 2026-06-10 -> D-012
The ST3215s are 360-deg servos: **no hard mechanical stop exists**; all limits are design judgment.
Current judgment: tibia **[-2.2, 0]** (folds one way, deep), recorded across URDF / ros2_control /
robot_params / ik_node. Residual follow-up lives in D-012: check link collision at full fold on the
physical build before driving real servos to -2.2.

## Q-002 — Physics simulator — RESOLVED 2026-06-10 -> D-010
Both: **Gazebo for Stage 2E** (gz_ros2_control, drop-in for our stack; add to Dockerfile) and
**MuJoCo/Isaac for RL** at Stage 5. Decided by Aryaman.

## Q-003 — Git commit/push policy — RESOLVED 2026-06-10 -> D-013
Commit per milestone on `stage-2`, push to origin after each, author Aryaman Gupta. SSH over 443.

---
## Non-blocking notes (track, resolve opportunistically)
## Q-010 — IK knee-bend direction — RESOLVED 2026-06-10 -> D-009
Visual check showed +1 folded the legs backward; default flipped to `knee_bend=-1` (forward fold).
Foot positions unchanged; tibia now stays within the servo's [-1.571, 0] range.

## Q-012 — Frame/label story — RESOLVED 2026-06-11 -> D-015
Aryaman, watching the physics walk in Gazebo, ruled: **forward = body +X** (the arc direction set in
the RViz reversal session). `forward_sign=+1`; cmd_vel +x drives the body toward +X — verified
(+0.44 m straight, level). Consequence: the FL/FR-named hips sit at the +X (front) end, so the URDF
leg labels MATCH physical quadrants — the long-deferred rename is unnecessary. The earlier
"head is at -X" mesh reading was wrong (treadmill-perception confusion in pinned-body RViz).

## Q-015 — Lidar selection: RPLidar A2M12 vs lighter/3D alternatives (research in flight)
Aryaman is leaning A2M12. Research subagent dispatched (specs/weight-on-2.45kg-robot, ROS2 Humble
driver, 2D-on-walking-robot mitigations, alternatives, sim-first Gazebo integration). Decide after
the report; then add the equivalent sensor to the SIM first (gpu_lidar + bridge + slam_toolbox).

## Q-014 — Exact body CoM coordinates (Aryaman, pending)
The base_link inertial origin is currently (0,0,0) = geometric center. Aryaman shifted the physical
CoM toward center and will measure exact coordinates; when they arrive, set them as the base_link
inertial `<origin xyz>` and re-check the D-016 stance trim (rear_raise may shrink if the CoM is
already forward). Until then the trim compensates empirically.

## Q-013 — Open-loop gait realizes ~40% of commanded speed in physics
cmd_vel 0.12 m/s -> ~0.047 m/s realized (straight, level). Expected for an open-loop trot under a
stiff velocity-loop actuator model (stance slip, no body-velocity feedback). Tuning levers: period,
duty, step length scaling, foot friction, and ultimately state feedback (2D+ estimator) or RL (Stage 5).
Not blocking; revisit when sim fidelity matters.

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
