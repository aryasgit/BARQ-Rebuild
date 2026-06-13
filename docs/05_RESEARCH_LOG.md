# BARQ — Research Log: Iterations, Overridden Decisions, Metrics

> Publication-grade record of HOW results improved: every superseded decision, why the override
> won, and the measured deltas. **Standing practice (Aryaman, 2026-06-11): update this after every
> experiment, metric, or decision override.** Companion to `02_DECISIONS.md` (what stands) — this
> file records what was *replaced* and what that bought us.

## 1. The headline metric series

Physics walk benchmark (Gazebo Fortress, cmd_vel +x = 0.12 m/s, ~10 s, flat ground):

| # | Model iteration | Distance /10 s | Direction | Yaw drift /10 s | Lateral | Settle-z error |
|---|---|---|---|---|---|---|
| 0 | Idealized kinematics, estimated masses, 10 N·m | 0.67 m | **WRONG (reversed)** | — | 5 mm | **+11 mm** anomaly |
| 1 | + exact URDF kinematics, stance inits (D-014) | 0.44 m | correct | — | 5 mm | **0.2 mm** |
| 2 | + measured masses 2.448 kg, 2.94 N·m caps | 0.52 m | correct | −0.031 rad | 21 mm | 0.2 mm |
| 3 | + rear_raise 0.02 stance trim (D-016) | **0.60 m** | correct | **−0.018 rad** | **0.4 mm** | 0.2 mm |

Settle-z error = |measured standing height − model prediction|: our model-fidelity metric.
It collapsed from 11 mm to 0.2 mm when the kinematic model became exact — and stayed there
through mass and posture changes, i.e., the residual physics is now explained by the model.

## 2. Overridden decisions — and what each override bought

### D-007 (idealized leg kinematics) → D-014 (exact URDF-true model)
- **Original claim:** clean link lengths; URDF offsets "sub-cm, negligible."
- **How it fell:** a 25-agent adversarial review (3 confirmed / 18 refuted findings) computed the
  true error: **3.4 cm at the front feet, front/rear asymmetric** (knee-x offsets fold into fake
  lengths; femur has a fixed 10.7° in-plane angle; lateral offset 0.0755 not 0.0465).
- **What the override bought:** walking direction flipped from *reversed* to *correct*; the 11 mm
  height anomaly vanished (0.2 mm); the startup −32 mm lurch vanished; support polygon symmetric.
- **Method that made it stick:** `test_exact_kinematics.py` asserts FK == rotation-matrix
  composition of the raw URDF origins to <1e-12 — the model class error is now structurally
  impossible to reintroduce.

### D-011 (forward = −X "head end") → D-015 (forward = +X, physics-ruled)
- **Original basis:** human perception of a *pinned-body* RViz animation + a mesh-orientation guess.
- **How it fell:** ground-contact physics made direction unambiguous; the human re-ruled on the
  *physical* walk. A `forward_sign` parameter (added in anticipation) made the flip one value.
- **Lesson for the paper:** treadmill-style kinematic visualisation inverts casual direction
  perception; embodiment questions need ground contact (or hardware) as the oracle. Bonus: the
  ruling proved the URDF leg labels match physical quadrants — a planned full-rename refactor
  (12 joints × 5 config layers) was cancelled as unnecessary.

### Estimated inertials (1.66 kg, 10 N·m) → measured (2.448 kg, 2.94 N·m)
- **Original basis:** CAD-era guesses; femur was 3× underestimated (its servos!); effort limit
  was a placeholder ~3.4× above the real ST3215 peak (30 kg·cm).
- **What the override bought:** counterintuitively, the 47% heavier robot walked **15% farther**
  per command (normal force → friction authority), and the sim now *proves* 5–10× stance-torque
  margin — an actuator-sizing claim backed by simulation rather than datasheet arithmetic.

### Q-001 (tibia limit "conflict") → D-012 (limits are design judgments)
- **Original framing:** URDF said ±1.57, servo map said [−1.571, 0] — which is "right"?
- **How it fell:** the team established the 360° servos have **no hard stops**; both numbers were
  judgments. Recording that fact (and choosing −2.2) unlocked the deep crouch and, later, the
  exact-model stance — the "conflict" was a category error.

### Uniform stance → D-016 (rear_raise load-forward trim)
- **Original:** all four feet at equal depth; human observed the rear visibly over-loaded.
- **Override:** rear legs extended 2 cm (raising, not front-dropping — front-drop would breach the
  tibia envelope, in-plane reach floor 0.1079 m at q3=−2.2). Result: +4.5° nose-down, **+15%
  distance, −42% yaw drift, −98% lateral drift**. The human's load-distribution eye, quantified.

### Also overridden along the way
- **Knee fold branch** +1 → −1 (D-009): visual check + the servo range [−1.571, 0] corroborated —
  the old branch commanded angles the hardware physically cannot reach.
- **Crouch height** 0.16 → 0.115 → **0.13** (D-012 → D-014/D-016): pushed down for stability, then
  partially back up when honest swing-clearance physics (foot-sphere radius, command staircase)
  demanded real lift margin. Deeper ≠ better once contact dynamics are honest.
- **Gazebo metapackage** → slim package set: the convenience metapackage broke the Jetson image
  (CUDA OpenCV file conflict); minimal dependencies are a deployment-correctness decision.

## 2b. Sim-perception iteration (2026-06-11)
| Probe | Result |
|---|---|
| Scan-plane leveling | odom->laser rotation ≈ 0.001 rad: the -4.5 deg mount counter-wedge exactly cancels the D-016 stance pitch — a *designed* prediction confirmed in physics |
| Lidar rate | 9.7 Hz (light) / ~7 Hz (full SLAM load); headless EGL rendering works on Tegra |
| SLAM output | 8x6 m room mapped: /map 161x121 @ 0.05 m, 1127 occupied / 16898 free cells after one fwd-arc-fwd lap |
| Failures->fixes | empty world mapped nothing (features required); gz model-prefixed TF frames broke the scan chain -> forced-frame odom_tf node |

## 2c. Autonomy iteration (2026-06-11)
| Probe | Result |
|---|---|
| v/omega composition | cmd (0.12 m/s, 0.3 rad/s) walked a clean closed circle R~0.4 m (=v/omega) — yaw authority validated in physics |
| First autonomous mission | NavigateToPose (1.6, 2.2): SUCCEEDED; final (1.548, 2.067), error 0.14 m; ~2.6 m around a pillar inflation zone |
| Stack | lidar -> slam_toolbox -> nav2 (NavFn + RegulatedPurePursuit @ 0.12 m/s cmd) -> trot gait |
| Defects found by live human scrutiny | missing cmd_vel deadman (fixed, 1 s timeout); RViz late-join durability; cross-container FastDDS /dev/shm data loss |

## 2d. Obstacle-course iteration (2026-06-11)
| Probe | Result |
|---|---|
| Course | 10x8 m unknown map: 1 m doorway, 3-pillar slalom, box field; one NavigateThroughPoses command, 2 waypoints, ~16 m |
| Outcome | **PHYSICALLY COMPLETED**: final pose (-3.48, 2.30) vs WP2 (-3.6, 2.4) = 0.157 m; mapped the course en route |
| Autonomous recovery | behavior_server log: `spin completed successfully`, `wait completed successfully` — mid-course stalls self-recovered (observed by Aryaman as "rotated wrong, corrected itself") |
| Dynamic speed | RPP cost+curvature regulation enabled LIVE mid-mission via dynamic params: 0.22 m/s open-space ceiling, auto-slow near obstacles/turns |
| Compute finding | full sim load (physics+SLAM+nav2+2 GUI renderers) on one Orin times out action handshakes -> goal silently lost. Sim-only problem (physics/GUI loads absent on real robot), but informed the mission protocol: no robot-side GUIs, robust action clients, verify by telemetry not exit codes |

## 2e. State estimator iteration (2026-06-11) — closing the last dishonest seam
| Probe | Result |
|---|---|
| Sim IMU | BNO085-class noise on `/imu/data` — the exact topic the Stage-3 hardware will fill (interface parity by construction) |
| Legged odometry v1 | stance-diagonal FK deltas + IMU yaw, 50 Hz; planar (x, y, yaw) |
| **Drift vs ground truth** | **0.075 m after a fwd+arc+fwd lap (~1.6 m path) ≈ 4–5% of distance** — inside the typical 1–10% band for kinematic legged odometry |
| Designed-feature-breaks-estimator bug | The D-016 rear_raise trim made "two lowest feet = stance" always select BOTH REAR legs -> per-leg cyclic motion averaged to zero -> odometry flatlined (0.0015 m vs 0.86 m truth). Fix: compare stance DIAGONALS (each holds one front + one rear leg, so the trim cancels). Locked by unit test |
| Lesson | Estimator assumptions must be audited against every gait/stance *feature*, not just the nominal gait — a stance trim is invisible to the gait but fatal to a naive contact heuristic |

## 2f. Sim actuation honesty + the swing-drag discovery (2026-06-11)
Goal: close sim-to-real gaps before hardware arrives ("borderline away from dropping the stack in").
Method: instrument first (`diagnostics/sim_actuation_probe.py` step mode; `sim_walk_metric.py`;
bag-based tracking via `analyze_track_bag.py` after live rclpy sampling proved starvation-prone on a
loaded Jetson — C++ bag recorder is immune), then change one truth at a time. Walks here: vx=0.15,
10 s, flat ground, fresh spawns.

| Probe | Result |
|---|---|
| Engine-side envelope check | `xacro … \| ign sdf -p`: effort 2.94 N·m x12, velocity cap x12, foot mu — all present in the SDF the engine actually loads (not just our URDF) |
| Step response, stock plugin | 0.3 rad knee step: rise 200 ms, peak vel exactly 3.00 rad/s = 10 x error -> the plugin's position loop is `vel = gain x update_rate x error`, gain 0.1 -> k=10/s, ~6x softer than an ST3215 |
| Velocity-cap enforcement | 1.2 rad ankle step: peak vel pinned at exactly the URDF cap during slew -> engine clips; cap tightened 5.24 -> 4.71 rad/s (0.222 s/60 deg @12 V spec) |
| Trot tracking, k=10/s | bag of 838 states/420 cmds: knee/ankle RMS 75-93 mrad, peaks ~200 mrad (11.5 deg) — the swing arc was never executed fully |
| Upstream bug | `position_proportional_gain` is a node parameter, but ign_ros2_control 0.7.x creates its node BEFORE installing `<parameters>` into rcl global args -> the knob is unreachable by config. Vendored + 3-line patch (external/gz_ros2_control, BARQ.patch) |
| Step response, k=60/s | rise 50 ms vs 51 ms theoretical pure slew at 4.71 rad/s; zero overshoot — textbook servo behaviour |
| Trot tracking, k=60/s | mean RMS 17.8 mrad (3.1x better); hips ~0 |
| Friction sweep (vx=0.15, 10 s) | mu 0.9: 0.071 m/s; mu 0.5: 0.074; mu 0.25: 0.067 — realized speed **mu-INVARIANT at ~45-49%** |
| The deduction | mu-invariance + stiffness-invariance kills both slip and lag hypotheses. Remaining mechanism: grounded swing feet DRAG forward against stance push; both forces Coulomb -> u/vx = (N_st-N_sw)/(N_st+N_sw), mu cancels. FK-from-bag confirmed feet lift the full 20 mm relative to the HIP — but front apex depth 0.110 m sits 2 mm from the 0.1079 m fold limit: ~20 mm is the kinematic clearance CEILING at this crouch, and trot heave eats it |
| Fix + map (fresh spawns) | smoothstep swing (forward travel mid-swing, ~zero-velocity touchdown): duty 0.50 -> 51% yaw -0.26; duty 0.55 -> 47% yaw -0.04 (straight); duty 0.60 -> 57-62% yaw +0.26..0.43. Speed/straightness trade exposed as `gait_duty`; default 0.6 (nav2 closes heading in missions) |

Overridden decisions (the publication thread):
- "Foot friction at Gazebo default" -> explicit, parameterized, swept; the sweep's *null result* was
  the key instrument: mu-invariance is the fingerprint that separates kinematic loss from frictional loss.
- "Sim servo = plugin default" -> spec-derived k=60/s via a vendored 3-line patch; the sim stiffness
  knob now exists to be MATCHED to the bench (st3215_diag measures the same step metrics).
- "Linear swing profile" (inherited from the first gait) -> smoothstep; +27% relative speed.
- Q-013's original hypothesis ("stance slip / no feedback") -> refuted by its own tuning levers:
  the friction lever did nothing, which is what solved it.

Lessons:
1. **A null result from a parameter sweep is evidence, not failure** — mu-invariance did more
   diagnostic work than any positive result this session.
2. **Verify at the engine, not the source**: `ign sdf -p` catches what URDF-level review cannot
   (and float repr means grep for the tag, not the value).
3. **Live sampling on a loaded SBC lies** (rclpy executor starvation produced plausible-looking
   garbage twice); record with the C++ bag recorder, analyze offline, and gate every metric on its
   own sample-rate sanity check.
4. **Config knobs can be load-bearing and dead**: the stiffness parameter existed, was documented,
   and could never be set — construction-order bugs hide where nobody integration-tests defaults.
5. **Between-run state is part of the experiment**: mid-stride teleports made every gait config look
   broken (collapses to -10%) until runs were isolated to fresh spawns.

## 2g. Servo torque budget — normal trot (2026-06-13)
Goal: per-servo torque vs the 2.94 N·m (30 kg·cm) cap during a normal gait (servo-sizing +
P4/P6 input). Method: Gazebo exposes the joint's TRANSMITTED-wrench torque (ground reaction
included) — added an effort state interface (gazebo-only) so it reaches /joint_states; recorded a
7 s / ~14-cycle steady walk (vx 0.15, duty 0.6), phase-binned. Full writeup +
figure: docs/research/2026-06-13-torque-budget.md.

| Quantity | Value | vs cap |
|---|---|---|
| Continuous load, worst RMS (RL ankle) | 1.31 N·m | 45% |
| Sustained cyclic peak (RL ankle, phase-mean) | 1.86 N·m | 63% |
| Transient foot-strike peak (all servos) | 2.95-3.08 N·m | ~100% |
| Continuous safety factor (cap / worst RMS) | 2.2x | — |

Findings: (1) load sits on the REAR legs (rear ankles ~2x front RMS) — the D-016 load-FORWARD
trim extends the rear legs, lengthening their moment arms, so they bear more TORQUE even with less
vertical force (stability bought at rear-ankle headroom; -> Q-017). (2) Foot-strike transients
reach the cap, worst on rear legs. Caveat: transmitted wrench = total structural torque (actuator
+ contact impulse), not pure motor demand; the >cap spikes are impact impulses reacted by the
structure, partly a rigid-contact artifact (no modeled foot/servo compliance) — sustained numbers
transfer, transient peaks are a conservative upper bound to re-measure on the bench (P3-03).
Lesson: the right torque question has TWO answers (continuous RMS vs impact peak); reporting only
the peak would have falsely flagged the servos as overloaded.

## 3. Methodology notes (for the write-up)
1. **Stage-gated bring-up with a fidelity metric at each gate** (RViz kinematics → mock control →
   IK round-trip 1e-9 → physics settle-error 0.2 mm) localises faults to one layer at a time.
2. **Adversarial multi-agent review before physics debugging**: 25 agents, 18/21 findings refuted —
   the 3 survivors were precisely the bugs; refutation pressure kept the signal pure.
3. **Reference-implementation tests** (URDF chain composition) beat spot-check tests: they pin the
   *model class*, not sample points.
4. **Cheap reversibility for human-ruled choices** (forward_sign, rear_raise, knee_bend as
   parameters): when the oracle is a human watching physics, make their ruling a one-value change.
5. **Honest actuation limits early**: fantasy torque caps would have deferred the actuator-margin
   question to hardware, where it costs broken parts instead of a sim rerun.

## 4. Standing practice
After every experiment / override / tuning change, append here: what was believed, what replaced
it, the measured delta, and the test that locks it in. `03_CHANGELOG.md` records *what changed*;
this file records *why the change was an improvement* — the publication narrative.
