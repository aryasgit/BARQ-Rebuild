# P2-02 — Assembly sequence (leg-by-leg, horn-at-midpoint discipline)

> Phase P2 · verified against repo @ 4ea53a0

## Objective
Four legs and the body assembled such that **every servo horn was attached with the servo held at
midpoint (2048) and the link physically at its URDF-zero orientation**, cables routed one-bus-per-
leg with verified slack at the extremes of motion, every leg back-drivable by hand. This is the
discipline file: the printed-frame part geometry is NOT in this repo, so the rules here are
written to survive any bracket shape — when a printed part fights the discipline, the part loses
(reprint), not the discipline.

> ### ⛔ RULE: NEVER power-move an unassembled or partially constrained linkage
> Torque may be ON only when (a) the servo is bare (no horn), or (b) the joint is fully
> assembled, screwed, and cable-cleared — and the move is a procedure step. In every
> intermediate state: `./st3215_diag.py torque <ID> off` **before** fingers go near.
> A part-fastened link under power is a pinch hazard, a projectile, and a horn-tooth
> misalignment machine. No exceptions, including "just a tiny nudge".

## Prerequisites
- **G2.1 + G2.2 passed** — 12 labeled, calibrated servos with committed YAMLs.
- Printed frame parts (body, 4× hip brackets, coxa/femur/tibia links, horns), fastener kit per
  P0-02 BOM, medium-strength threadlock, zip ties / printed cable clips, paint pen.
- Bench rig from P2-01 (driver board + 12.0 V supply + `st3215_diag.py`), two terminals.
- Phone inclinometer app (any; verify it reads 0.0° on a known-level surface first) or a printed
  protractor jig.
- A build card (paper/phone note) per leg.

## What "link at URDF-zero" MEANS — derive once, tape it to the bench
At all-q = 0 the URDF leg hangs straight down. Exact invariants (computed from `fk_exact` /
`barq.urdf.xacro` joint origins at @4ea53a0):

| Link | URDF-zero orientation (physical) | Inclinometer target | Tol |
|---|---|---|---|
| **coxa** (hip joint, axis = body X) | neutral roll: the knee axis is horizontal, the leg plane vertical | knee-servo mounting face / knee-axis line: **level, 0.0°** | ±2° |
| **femur** (knee joint, axis = body Y) | the knee-axis → ankle-axis line leans **10.7° toward body-front (+X)** from plumb — same lean for all four legs (ankle origin (0.018944, ·, −0.1) m in the femur frame) | **79.3° from horizontal** along the joint-to-joint line; if you must measure a flat printed face instead, FIRST record that face's own angle to the joint line (T2.6) | ±2° |
| **tibia** (ankle joint, axis = body Y) | tibia long axis (ankle axis → foot tip) **plumb** | **90.0°** | ±2° |

Whole-leg cross-check at zero (tape measure): kinematic foot point **0.200 m vertically below
the hip axis** (physical foot tip ≈ 0.212 m with the 12 mm contact sphere), and **36 mm forward
of the hip for FRONT legs / ≈ 1.5 mm for REAR** (knee_x ±0.01744 + ankle_x 0.018944 — fk_exact,
repo-verified; an older note quoting ≈0.207 m is superseded by the code).

**Tolerance honesty:** ±2° is the *jig reading* tolerance. The horn spline has a finite tooth
count (not in the repo — count it on the first horn, T2.7): the nearest-tooth residual can
legitimately exceed 2°. Discipline: seat at the **nearest tooth**, MEASURE and record the actual
link angle on the build card, and let P2-03's `zero_offset` absorb the residual in software.
Re-clock ±1 tooth only when that reduces |residual|. Software can absorb any *constant, recorded*
offset; it cannot absorb slip or guesswork.

## The horn-attachment ritual (same 6 steps, every joint)
1. Servo already mounted in its bracket, horn OFF, its cable segment already routed (below).
2. Power the bus. Hold midpoint: `./st3215_diag.py move <ID> 2048 --speed 200` → expect final
   `err` within ±3 counts. Torque is now ON and holding.
3. Offer up horn+link at the URDF-zero orientation from the table (inclinometer on the link),
   seat on the spline at the nearest tooth.
4. Second terminal: `./st3215_diag.py monitor <ID> --hz 10` while you press and screw — `pos`
   must stay **2048 ± 10** throughout. If it walks, you torqued the shaft: pull off, re-seat.
5. Horn screw in (threadlock per the screw discipline below); link fasteners snug.
6. Jig-measure the final link angle, write it on the build card, then
   `./st3215_diag.py torque <ID> off` before any further handling.

## Order of operations per leg (FL → FR → RL → RR; finish + verify one leg before the next —
a discipline error caught on leg 1 must not be ×4)
1. **Dry-fit** the whole leg without screws (and without servos where possible): part fit, screw
   paths reachable, no forced flex, horn screw access confirmed at the *assembled* orientation.
2. **Coxa stage:** hip servo (ID per the P2-01 map) into the body-side bracket → ritual 1–6 with
   the coxa link.
3. **Femur stage:** knee servo into the coxa-side bracket → ritual 1–6, jig on the 79.3° line.
4. **Tibia stage:** ankle servo into the femur-side bracket → ritual 1–6, tibia plumb.
5. **Cable pass** finalized at each stage BEFORE the next stage closes access.
6. **Leg verification** (G2.3/G2.4 checks below) before starting the next leg.

## Cable routing per bus (bus = leg; daisy-chain hip → knee → ankle, board/Teensy end at the hip)
- Cables cross **joint axes**, not link spans; service loop at each joint on the axis side; never
  across a pinch line (the tibia fold is a designed pinch hazard — Doomsday §3).
- **Lengths are TBD-measure (T2.8):** with torque OFF, move every joint slowly BY HAND through
  its FULL design range — coxa ±45°, femur ±90°, tibia 0 → −126° (stop at first interference;
  note the angle — that observation feeds P2-03's fold test) — while watching the cable.
  **Slack rule:** ≥ 10 mm slack remaining at the tautest extreme, no contact with moving edges,
  zero tension at any connector. Record the final cut/loom length per segment in T2.8 (this
  becomes the spares-kit spec).
- Strain relief within ~30 mm of every connector — zip tie to an anchor point, never around the
  free bundle alone. Latch every connector; paint-pen witness mark across each housing.
- Bus sanity after closing each leg: `./st3215_diag.py scan` → exactly that leg's 3 IDs, e.g.
  FL: `3 servo(s): [0, 1, 2]` with the plan names printed.

## Screw & threadlock discipline
- Metal into metal (horn screws, machine screws into inserts): ONE drop of medium-strength
  threadlock.
- Screws into printed plastic: **no threadlock** (solvent attack / cracking) — torque to
  "snug + 1/8 turn", never more; a stripped boss = reprint or heat-set insert, not a longer screw.
- Paint-pen witness mark on every torqued screw head; the P4 runbook re-checks marks after the
  first hour of operation.

## Battery bay & CoM note
- The 4S GENX 5200 (512 g) is **already inside** the 1420 g body mass in the URDF/robot_params —
  do not add its mass anywhere again.
- Mount with a strap/retainer: the pack must not shift (it is ~21 % of total mass — a moving CoM
  invalidates everything tuned on the sim). Leave access: **Q-014 (CoM measurement) is executed
  with the battery installed**.
- Weigh the finished robot (T2.9): expect ≈ **2.448 kg** (robot_params link masses + body).
  A > 5 % miss means the model is lying somewhere — research-log entry before proceeding.

## Per-leg verification (feeds the gates)
1. Torque off all three servos (`torque <ID> off` ×3) — verify with `status` (`torque OFF`).
2. **Back-drive (G2.3):** hand-move each joint smoothly through its FULL design range with
   `./st3215_diag.py monitor <ID> --hz 10` running: counts change smoothly, no dropouts, no
   notchy spots, no link↔link or link↔servo-case contact anywhere in coxa ±0.785 rad /
   femur ±1.57 rad / tibia [−2.2, 0] (tibia: stop at first contact and write down the counts —
   that is P2-03 fold-test data, not a failure).
3. **Cable extremes (G2.4):** at each range extreme, tug-test: ≥ 10 mm slack, no connector
   strain; wiggle the loom while `monitor` runs — zero dropouts.
4. `scan` shows the leg's 3 IDs instantly.

## TBD table (measure, never guess)
| TBD | Value | Measurement procedure |
|---|---|---|
| T2.6 | femur flat-face ↔ joint-line angle | one-time on the first femur: measure both the face and the joint-to-joint line, record the difference; thereafter jig on the face |
| T2.7 | horn spline tooth count → half-pitch residual bound (= 180°/teeth) | count teeth on the first horn before mounting |
| T2.8 | cable segment lengths per bus | slack-rule procedure above, per segment, all 4 buses |
| T2.9 | as-built robot mass | weigh complete robot incl. battery; compare 2.448 kg model |

## Acceptance gates
- **G2.3 — every assembled leg back-drivable by hand** through its design range: smooth monitor
  trace, no binding, no collision (tibia: first-contact angle recorded instead). All 4 legs.
- **G2.4 — cables unstressed at range extremes:** slack rule met at every joint of every bus;
  scan stays clean during the wiggle test.

## Fallback ladder
- **A** — discipline as written. *Switch when:* the same joint fails seating/alignment twice.
- **B** — re-clock the horn ±1 tooth and retry; if the bracket itself prevents the target
  orientation, relieve ONLY non-structural interference (file/deburr) and record the deviation
  on the build card. *Switch when:* 2 attempts or 2 h on one joint.
- **C** — reprint the offending part (this is a P0 print-queue loop, not a P2 failure); continue
  on the other legs meanwhile — legs are independent up to the body mount.

## Rollback
Disassemble in exact reverse order. Any servo that came out of a leg goes BACK TO THE P2-01
BENCH (status + calibrate-mid spot-check + one sweep) before reuse — assembly stress is exactly
the failure you want to catch before it is buried in a leg again. Never store a half-assembled
leg with torque on.

## Artifacts → docs/05_RESEARCH_LOG.md
- Build card per leg (horn residual angles, screw witness map, first-contact tibia counts) —
  photograph and attach.
- T2.6–T2.9 rows filled; per-leg session entry (believed / measured / changed);
  `docs/01_STATUS.md` one-liner; commit, push (D-013).

## If this entire phase approach fails
If the printed frame fundamentally cannot mount horns at midpoint/URDF-zero — bracket geometry
forcing offsets beyond ±(half tooth + 2°) on multiple joints, or links colliding inside the
design range on every leg — STOP assembling. The fix is upstream: re-CAD the horn interfaces /
links, reprint, and re-enter this file. A single stable, measured offset on a single joint may
be assembled anyway and absorbed by P2-03 `zero_offset` (record it); systemic misfit may NOT be
absorbed in software. If the frame is unavailable for an extended period: the bench program
continues without it — P3 firmware bring-up needs only one servo on the P2-01 bench, and P6-01/02
run in sim (Doomsday §5 dependency graph).
