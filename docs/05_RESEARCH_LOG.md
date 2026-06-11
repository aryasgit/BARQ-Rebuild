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
