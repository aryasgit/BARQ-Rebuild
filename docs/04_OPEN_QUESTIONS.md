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

## Q-015 — Lidar selection · report in (docs/research/2026-06-11-lidar-selection.md), decision pending
Research verdict: A2M12 works (best driver story) but is the wrong mass class for a 2.45 kg robot
(190 g high-mounted ≈ 9% body mass, ~$229). **Recommended: LDROBOT STL-27L** (~45 g, 25 m,
21.6 kHz, ~$142, strictly better specs) or LD19/D500 budget (~47 g). 3D Unitree L1 deferred.
Either way: nose depth camera later for the below-plane hazard zone (everything under 0.22 m).
**Jetson prep found live: `sudo apt remove brltty` (CP2102 hijacker) + add user to dialout.**
Sim integration plan is lidar-agnostic and ready (§5 of the report) — build it now.

## Q-014 — Exact body CoM coordinates (Aryaman, pending)
The base_link inertial origin is currently (0,0,0) = geometric center. Aryaman shifted the physical
CoM toward center and will measure exact coordinates; when they arrive, set them as the base_link
inertial `<origin xyz>` and re-check the D-016 stance trim (rear_raise may shrink if the CoM is
already forward). Until then the trim compensates empirically. NOTE (2026-06-13): measure with the
battery INSTALLED — the 4S pack (512 g, D-021) is part of the 1420 g body mass and dominates CoM;
rod-balance procedure in roadmap P4-01.

## Q-013 — Open-loop gait realizes ~40-50% of commanded speed — RESOLVED 2026-06-11 -> D-019
Root cause found by elimination: invariant to foot friction (mu 0.25-0.9 sweep) AND to actuator
stiffness (k=10 vs 60/s) -> not slip, not lag — swing-foot DRAG (grounded swing feet slide forward
against the stance push; both forces Coulomb, so mu cancels — that invariance IS the fingerprint).
Front clearance is reach-capped at ~20 mm and trot heave consumes it. Swing reshaping + duty 0.6
recover ~60% realized (D-019); the residual gap is heave-driven intermittent contact. Full fix
needs body-state feedback in the gait (estimator exists) or RL (Stage 5).

## Q-016 — Open-loop yaw bias flips sign with gait duty (zero-crossing ~0.55)
Straight-line 10 s walks, fresh spawns: duty 0.50 -> yaw -0.26 rad (right veer, 51%); duty 0.55 ->
-0.04 rad (straight, 47%); duty 0.60 -> +0.32 +/- 0.06 rad (left veer, ~60%). Mechanism unknown
(double-support overlap timing x diagonal-pair chirality?). Missions are unaffected — nav2 RPP
closes the heading loop (obstacle course completed with worse) — so default stays duty 0.6 for
speed; `gait_duty:=0.55` for straight open-loop demos. Candidate proper fix: estimator yaw-rate
feedback into `wz` in the gait planner (~20 lines); measure both before/after for the log.

## Q-017 — Rear-leg torque headroom: the D-016 load-forward trim costs rear-ankle torque
Torque-budget (research/2026-06-13-torque-budget.md): rear ankles bear ~2x the front ankles' RMS
(RL ankle 1.31 vs FL 0.72 N·m) and the sustained cyclic peak reaches 1.86 N·m (63% of cap) on the
RL ankle. Mechanism: D-016 shifts the vertical LOAD forward by EXTENDING the rear legs, which
lengthens their moment arms -> more torque despite less force. Not blocking (2.2x continuous
margin) but it sets where to watch first on hardware (rear ankle/knee current at touchdown, P4)
and what the RL torque penalty should target (P6). Revisit alongside Q-014 (exact CoM): if the
real CoM is already forward, rear_raise can shrink, recovering rear-leg headroom. Re-measure the
*transient* peaks on the bench (P3-03) — they are partly a rigid-contact sim artifact.

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
