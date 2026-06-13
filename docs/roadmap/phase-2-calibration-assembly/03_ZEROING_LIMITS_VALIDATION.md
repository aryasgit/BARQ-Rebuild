# P2-03 — Zeroing, direction & limits validation (incl. the D-012 fold test)

> Phase P2 · verified against repo @ 0e5ddaf

## Objective
Software-true zero for the assembled robot: per-joint `zero_offset` and `direction` measured and
committed into `robot_params.yaml`; the **D-012 obligation discharged** (physical tibia fold test
to −2.2 rad, unpowered, all four legs, before anything ever drives there powered); per-servo
EPROM backstop limits written; whole-robot pose checks on the cradle that visually match the sim.
Exit of this file = exit of Phase P2.

## Prerequisites
- **G2.1–G2.4 passed.** Robot fully assembled per P2-02; build cards at hand.
- **Cradle:** robot suspended, feet free, body level. Master power kill within arm's reach
  (Doomsday §3). Battery not required — the P2-01 bench 12 V supply is fine.
- Bench driver board reachable to each leg's bus (move it leg-to-leg if there is only one).
- Inclinometer jig + the URDF-zero table from P2-02.
- The repo checked out and the sim container working (the regression in step 4 runs in sim).
- Two terminals; this conventions sheet printed.

## Sign convention table (REP-103: x forward, y left, z up — derived from the URDF axes, repo-verified)
| Joint (servo) | URDF axis | **+q means, physically** | Stance value (URDF init) |
|---|---|---|---|
| `*_hip_joint` (coxa) | +X | foot arcs toward **body-LEFT (+Y)** — same sign for all 4 legs | 0.0 |
| `*_knee_joint` (femur) | +Y | foot end swings toward **body-REAR (−X)** | front **+1.047531** / rear **+0.911998** |
| `*_ankle_joint` (tibia) | +Y | same sense as knee; working range is the NEGATIVE side **[−2.2, 0]**: more negative = deeper **forward** fold (foot toward +X relative to the femur) | front **−1.928768** / rear **−1.652637** (D-016 rear_raise trim) |

Mirroring is absorbed at the hardware layer (D-001): the +q definitions above are identical for
left and right; the `direction` multiplier makes the hardware obey them. The current yaml has
FR_coxa/RR_coxa = −1 — treat as a **hypothesis** until step 2 measures it.

**Servo-frame conversion** (this DEFINES what P3's hardware interface must implement — copy it
into `docs/06_PROTOCOL.md` when the P3 stubs are filled):

```
counts(q)        = 2048 + direction · (q − zero_offset) · 651.90     [651.90 = 4096/2π counts/rad]
deg-for-move(q)  =        direction · (q − zero_offset) · 57.2958    [st3215_diag move --deg is absolute from 2048]
zero_offset      := the URDF-frame angle the link actually sits at when the servo reads exactly 2048 (rad, signed per the table)
```

## Procedure

### 1. Per-joint zero check → `zero_offset` (one leg at a time, plan order ID 0 → 11)
1. `./st3215_diag.py move <ID> 2048 --speed 200` (slow). Expect final `err` within ±3 counts.
2. Jig-measure the link's true orientation against its URDF-zero target (P2-02 table: coxa level,
   femur joint-line 79.3°, tibia plumb). Convert the deviation to **radians, signed per the
   convention table** (e.g. a tibia hanging 3° toward body-front of plumb = q is negative-side
   = zero_offset −0.052).
3. Write it into `barq_description/config/robot_params.yaml` → `servos: <name>: zero_offset`.
4. Sanity: |zero_offset| should be ≤ (half horn-tooth pitch + 2° jig) per T2.7 — it equals the
   residual recorded on the P2-02 build card. **If > 0.15 rad: that is a mechanical problem —
   re-seat the horn (P2-02 ritual), do not paper it over in software.**
5. `torque <ID> off` before handling the leg between joints.

### 2. Direction verification per joint
From mid: `./st3215_diag.py move <ID> --deg 5.7 --speed 200` (= **+0.1 rad in the servo frame**,
absolute from center). Watch the link against the convention table:
- moves the +q way → `direction: 1`
- moves the −q way → `direction: -1`

Return with `./st3215_diag.py move <ID> 2048 --speed 200`, then record in `robot_params.yaml`.
Hips: watch the **foot** arc with feet free — toward +Y (body-left) is +. Expected if the build
matches the CAD assumption: FR_coxa and RR_coxa = −1, all others +1 — but **the measurement
wins**; any difference is a research-log entry superseding the yaml hypothesis (D-001 mechanism
itself is unchanged).

### 3. D-012 fold test — UNPOWERED, all four tibias (gate G2.6)
D-012's standing obligation: the ST3215 has no mechanical stop; [−2.2, 0] is a design judgment
that must be physically verified for link/servo-case collision **before** ever driving there
powered. The in-plane reach floor at q3 = −2.2 is 0.1079 m (D-016/D-019) — this test decides
whether that floor is real.

Per leg (FL, FR, RL, RR):
1. `./st3215_diag.py torque <ID> off` for all 3 servos of the leg; confirm via `status`.
2. Run `./st3215_diag.py monitor <ID_tibia> --hz 5` (position still reads with torque off).
3. **Hand-fold the tibia slowly** from 0 toward −2.2 rad. Servo-frame target for reference:
   `counts(−2.2)` — with direction +1, zero_offset 0 that is **614 counts (−126.05°)**. One hand
   folds; eyes on every closing gap: tibia↔femur, tibia↔servo case, horn screws, cable loop.
4. **STOP at first contact or interference.** Read the counts from `monitor`; convert:
   `q_contact = direction·(counts − 2048)/651.90 + zero_offset`.
5. `safe_min := q_contact + 0.05 rad` (≈ 3° standoff margin — judgment, recorded as such).
6. Fill the table — this table IS the G2.6 deliverable (T2.10):

   | Leg | contact counts | q_contact (rad) | safe_min (rad) | what touched |
   |---|---|---|---|---|
   | FL | | | | |
   | FR | | | | |
   | RL | | | | |
   | RR | | | | |

7. **Decision rule** (one limit governs all four legs — control is symmetric, Option A/D-001;
   take the *shallowest*, i.e. `q3min := max(safe_min over legs)`):
   - All `safe_min ≤ −2.2` → the design limit stands; no file changes; record the table and move on.
   - **Any `safe_min > −2.2`** (shallower than design) → execute step 4 with the new `q3min`.
8. **Consequence pre-check before editing anything.** Recompute the in-plane reach floor:

   ```
   R(q3min) = sqrt(l2p² + l3² + 2·l2p·l3·cos(q3min + 0.18717))      l2p = 0.101779, l3 = 0.1
   ```
   (Check: R(−2.2) = 0.1079 m — matches D-016/D-019.) The gait swing apex sits at depth
   stand_height − step_height = 0.13 − 0.02 = **0.110 m** (D-019), only 2.1 mm above the current
   floor. **If q3min is shallower than ≈ −2.18 rad, R(q3min) > 0.110 m and the current gait
   geometry is broken** — you must also retune `stand_height`/`step_height` (gait_planner/ik_node
   params) until apex depth ≥ R(q3min) + 0.002 m, then re-baseline the walk metric (step 4c).

### 4. Limit update — exact file list (only if step 3.7 demands; ONE commit, all files together —
partial updates create URDF ≠ yaml ≠ clamp skew, the exact bug class D-012 exists to prevent)

| File | What changes |
|---|---|
| `barq_description/urdf/barq.urdf.xacro` | 4× ankle `<limit effort=... lower="-2.2" ...>` lines (FL/RL/FR/RR `_ankle_joint`) **and** 4× `<xacro:barq_joint_if ... _ankle_joint" lower="-2.2" ...>` lines in the ros2_control block → new q3min |
| `barq_description/config/robot_params.yaml` | 4× tibia `min_angle: -2.2` rows |
| `barq_control/barq_control/ik_node.py` | module constant `ANKLE_MIN = -2.2` (≈ line 29) |
| `barq_control/barq_control/gait.py`, `gait_planner_node.py` | the comment lines citing −2.2 / 0.1079 / the stand−step constraint — update the numbers so the next reader isn't lied to |
| `docs/02_DECISIONS.md` | dated amendment under D-012 with the measured table |

> **KNOWN INCONSISTENCY @0e5ddaf — fix in the same commit:** the URDF ankle **upper** limit is
> `1.57` in both the `<limit>` lines and the ros2_control `barq_joint_if` lines, while
> robot_params (`max_angle: 0.0`) and ik_node (`ANKLE_MAX = 0.0`) say the design max is **0**.
> D-012 claims all four layers agree — the URDF upper does not. Set it to `0.0` while editing
> these lines (stance/gait ankles are always negative, so behavior should not change); if the
> sim regression below objects, record, revert only that, and open a question in
> `docs/04_OPEN_QUESTIONS.md`.

Then the **sim regression** (in the container, per P0):
- a. `colcon test --packages-select barq_control` → kinematics/IK/gait suites green
  (`test_exact_kinematics.py`, `test_ik.py`, `test_gait.py`).
- b. Launch the sim (`barq_bringup` `sim.launch.py`) and run
  `python3 diagnostics/sim_walk_metric.py --vx 0.12 --duration 10` → compare the research-log
  headline line (0.60 m /10 s, yaw drift −0.018 rad, lateral 0.4 mm class). Degradation > 20 %
  means the new limit materially shrank the workspace → gait re-tune loop (knobs per D-019:
  `gait_duty`, `stand_height`, `step_height`), new research-log line.
- c. `python3 diagnostics/sim_actuation_probe.py track --duration 12` → per-joint tracking in
  the D-018 family (mean RMS ~17.8 mrad class, no joint pinned at a clamp).

### 5. EPROM backstop limits (hardware backstop UNDER the software clamps — defense in depth)
Now that `direction` and `zero_offset` are measured, write each servo's EPROM angle limits:
design range widened by 30 counts of slack each side, **except the tibia fold side, which gets
exactly `counts(safe_min)`** — the hardware backstop sits ON the measured physical truth.

```
lo = counts(q_min),  hi = counts(q_max)        # via the conversion formula, per servo
if direction = −1 the numeric order swaps → write  --min min(lo,hi)  --max max(lo,hi)
```

Nominal values for direction +1, zero_offset 0 (recompute per your measured yaml — do not
copy blindly):
| Joint class | design range (rad) | EPROM `--min` | EPROM `--max` |
|---|---|---|---|
| coxa  | ±0.785 | 1506 | 2590 |
| femur | ±1.57  | 995  | 3101 |
| tibia | [safe_min, 0] | counts(safe_min) | 2078 |

```
./st3215_diag.py limits <ID> --min <lo> --max <hi>
./st3215_diag.py limits <ID>          # read-back verify; record in the servo's calib YAML (T2.13)
```
Clamp demo on ONE tibia (cradle, leg free): command 10° beyond the fold side via the formula
(`move <ID> --deg <...> --timeout 2`) → the move must stop at the EPROM bound with a large final
`err` — that is the backstop working. Return to `move <ID> 2048`.

### 6. Whole-robot powered pose checks on the cradle (gate G2.5)
Safety: cradle, fingers OUT of the leg workspace while torque is on (Doomsday §3), kill within
reach. One leg at a time first, then all four.

1. **Zero pose.** Per leg, in hip → knee → ankle order:
   `./st3215_diag.py move <ID> 2048 --speed 100 --acc 20`. Visual: leg straight down, tibia
   plumb; tape measure: foot ≈ 0.200 m below the hip axis (±5 mm), front feet ~36 mm ahead of
   the hip, rear ~at the hip (P2-02 table).
2. **Stance pose** (= the URDF ros2_control init values). Command knee then ankle (hips already
   at 0) with `--speed 100 --acc 20`. Targets — nominal counts for direction +1, zero_offset 0;
   **recompute via the formula with your measured yaml values**:

   | Joint | q (rad) | servo deg | counts |
   |---|---|---|---|
   | all hips (0,3,6,9) | 0.0 | 0.0 | 2048 |
   | front knees (1,4) | +1.047531 | +60.02 | 2731 |
   | front ankles (2,5) | −1.928768 | −110.51 | 791 |
   | rear knees (7,10) | +0.911998 | +52.25 | 2643 |
   | rear ankles (8,11) | −1.652637 | −94.69 | 971 |

   Note on "slow interpolation": the diag tool has no synchronized multi-joint interpolator —
   the low `--speed/--acc` trapezoid inside the servo IS the interpolation at P2, and joints are
   sequenced one at a time. Coordinated 100 Hz interpolation arrives with the Teensy in P3.
3. **Current/load table.** At stance, `./st3215_diag.py status <ID>` for all 12; record mA and
   load % per servo (T2.11). On the cradle (feet free) legs carry only their own link weight:
   currents should be small and symmetric across legs.
4. **Asymmetry rule:** same-joint-class current imbalance **> 30 %** across legs (e.g. FL_femur
   vs the other three femurs), or any servo trending hot → **mechanical binding ladder**:
   (1) torque off → repeat the G2.3 back-drive on the suspect leg by hand;
   (2) cable snag check at that pose (G2.4 extremes recheck);
   (3) horn/bracket over-constraint — loosen-retighten the stage's screws in pattern;
   (4) swap-test the servo with a bench-passed spare (Doomsday §4.4) — if the imbalance follows
   the servo it's the servo; if it stays with the leg it's the mechanics.
5. **Visual match vs sim** (the other half of G2.5): launch the sim/RViz view —
   `ros2 launch barq_bringup visualize.launch.py` (mock mode boots at the URDF init values
   = the stance trim; `sim.launch.py` equally valid) — screenshot the side view (via the VNC
   path from P0 if headless). Photograph the cradled robot from the same side. Side-by-side:
   front hip→foot 0.13 m vs rear 0.15 m (the D-016 nose-down rake when standing), knee/ankle
   fold visually congruent. File both images.
6. **Cycle endurance mini-check:** stance → zero → stance ×3 at `--speed 60`, watching
   `monitor` on a front ankle (largest travel, 110°) and the current; temps at the end ≤ the
   T2.2 bench bar; currents repeat cycle-to-cycle (drift = something is winding up).

## TBD table (measure, never guess)
| TBD | Value | Measurement procedure |
|---|---|---|
| T2.10 | fold-test table (4 legs) | procedure step 3 — the G2.6 deliverable |
| T2.11 | stance current baseline per joint class on cradle (mA) | step 6.3 — becomes the P4 health-monitor baseline |
| T2.12 | final zero_offset + direction per joint | steps 1–2; the robot_params.yaml commit is the record |
| T2.13 | EPROM limits as written per servo | step 5 read-back → `eprom_limits_written` field in each `servo_calib/*.yaml` |

## Acceptance gates
- **G2.5 — zero committed + stance matches sim.** `zero_offset` and `direction` for all 12
  joints committed in `robot_params.yaml`; stance photo vs sim screenshot filed side-by-side
  with congruent rake/fold; stance current table recorded with no unexplained > 30 % imbalance.
- **G2.6 — fold-test table recorded for all 4 tibias** (T2.10) **+ the decision** — either
  "−2.2 stands" or the step-4 single commit with the sim regression green.

## Fallback ladder
- **A** — as written. *Switch when:* one joint refuses to zero or direction-verify twice.
- **B** — re-bench the suspect servo solo (P2-01 steps 5–9 on the bench board): separates a
  servo fault from an assembly fault. *Switch when:* 2 h.
- **C** — re-open P2-02 for that joint (horn re-seat ±1 tooth, bracket check), then re-run this
  file for that leg only.
- **Pose-check ladder:** if EVERY leg shows imbalance simultaneously, suspect the 12 V supply
  sagging under 12 servos before blaming four mechanisms at once — multimeter at the board
  under load; supply current limit; that is P1 territory (fix there, then return here).

## Rollback
- First action on anything unexpected: torque off all twelve (`torque <ID> off` ×12).
- `robot_params.yaml`: `git checkout`/revert — the yaml values are the only mutable state.
- EPROM limits: restore each servo from its as-found values (T2.4 in the P2-01 YAML).
- URDF/ik/gait limit edits: revert the single step-4 commit (that's why it is ONE commit).

## Artifacts → docs/05_RESEARCH_LOG.md
- `robot_params.yaml` diff (zero_offset/direction) + updated `servo_calib/*.yaml`
  (`direction_observed`, `eprom_limits_written`) committed.
- Fold-test table, stance current table, photo + sim screenshot pair.
- Research-log entry (believed / measured / changed — the fold test result is publication
  material either way), `docs/01_STATUS.md` one-liner, `HANDOFF.md` frontier, commit, push.

## If this entire phase approach fails
If zeroing will not converge — offsets non-repeatable across power cycles, positions drifting —
suspect the magnet/encoder or an EPROM `ofs` wrap: re-run P2-01 `calibrate-mid` on the offender;
two or more offenders → re-bench all twelve before trusting anything. If the fold test reveals
collisions FAR shallower than −2.2 on all legs (workspace collapse), the printed geometry
diverges from the URDF: STOP — re-measure the physical links, re-derive the URDF origins, and
accept that downstream sim results need re-validation (scope it in `docs/04_OPEN_QUESTIONS.md`
+ research log). In either stall the robot stays on the cradle and the program continues in
parallel: P3 firmware bring-up needs only the single-servo bench, and P6-01/02 run in sim
(Doomsday §5). Do not let a zeroing stall become a schedule stall.
