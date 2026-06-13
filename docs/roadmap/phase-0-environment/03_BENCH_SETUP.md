# P0-03 — Physical bench: stations, test cradle, safety, labels

> Phase P0 · verified against repo @ 0e5ddaf

## Objective
A bench that makes the hardware phases boring: a servo bench station wired exactly as
`diagnostics/README.md` expects, a LiPo handling corner that can't burn the room down, a
test cradle that holds the robot with **feet free** (P3 air-walk and P4 first-stand both
require it — protocol §3: cradle first, no exceptions), label discipline so bus/ID mistakes
become impossible, and the right instruments in reach.

## Prerequisites
- `02_BOM_PROCUREMENT.md`: bench power (B10 any rung), connectors/wire/fuses (B5–B7),
  USB-UART if needed (B9/TBD-P0-6). LiPo rows (B3/B4) before the battery enters the room.
- `diagnostics/README.md` read fully — this file places hardware; that file runs it.
- ~2 × 1 m of table against a wall + ~0.5 m² of floor or shelf for the cradle.

## Procedure

### 1. Bench layout (left→right = dangerous→delicate)
```
   WALL ──────────────────────────────────────────────────────────────────────
   │ [A] LiPo ZONE        [B] POWER          [C] SERVO BENCH       [D] DESK   │
   │  ceramic tile/       bench PSU          anti-slip mat         Jetson +   │
   │  metal tray          master switch      driver board(s)       monitor    │
   │  LiPo bags           fuse panel         USB-UART → Jetson     or VNC     │
   │  sand bucket         (arm's reach       servo under test      laptop     │
   │  NOTHING else        from C and D)      vice / clamp          docs       │
   │  ≥50 cm clear        12V + 5V rails     label maker           printed    │
   ───────────────────────────────────────────────────────────────────────────
                                  [E] CRADLE (floor or sturdy shelf, near C)
```
Rules baked into the layout: the LiPo zone [A] holds batteries and nothing else; the master
switch in [B] is reachable from wherever your hands are when torque is on (protocol §3);
the servo under test in [C] is clamped, never hand-held, during sweeps.

### 2. Servo bench station (per-servo hookup chain)
Wire exactly this chain — it is the `diagnostics/README.md` bench, do not improvise:
```
[bench PSU 12.0 V, current-limited]
      │  (16 AWG, inline fuse, XT30)
      ▼
[Waveshare driver board] ──USB──► [Jetson host]          # if O2 board has onboard USB
        │                  └─ else: 3.3 V TTL ──► [CP2102/CH340 USB-UART] ──► Jetson
        │  (3-pin daisy-chain lead)
        ▼
[ST3215 under test]  (clamped; NO horn/linkage for first power-up — diagnostics README §Safety)
```
- Power the driver board from its **own supply, never USB** (diagnostics README). The
  README's "7.4 V-class" note is the servo's wide rated range; **the robot's rail is
  12.0 V, so acceptance sweeps run at 12.0 V** — calibrating temps/currents at 7.4 V would
  not represent flight. First-contact trick: PSU current limit ~1 A until the servo proves
  sane, then open it up.
- Host prep must already be done: `brltty` removed, `dialout` group (P0-01 step 2). Device
  appears as `/dev/ttyACM0`/`/dev/ttyUSB0`; the tool auto-detects or takes `--port`.
- Run the per-servo procedure from `diagnostics/README.md` (plan → scan → set-id →
  status → calibrate-mid → sweep → torque off). That file is the procedure of record —
  this one only builds its table.
- An INA260 in series on the 12 V feed is the bench ammeter until a clamp meter exists —
  this is how TBD-P0-1/2 (run/stall current) get measured in P1.

### 3. LiPo handling station (zone [A])
Protocol §3 is binding; station specifics on top of it:
1. Charge ONLY here: pack in LiPo bag, bag on ceramic tile/metal tray, sand bucket within
   reach, charger profile matching VERIFIED chemistry (TBD-P0-4: 4.20 vs 4.35 V/cell —
   unverified pack = no charge, full stop).
2. Charging is supervised — same room, line of sight. No overnight charging, ever.
3. Storage-charge (≈3.8 V/cell) if idle > 3 days; floor 13.6 V (3.4 V/cell) in use.
4. Damaged/puffed/smelly pack: outdoors immediately, 30 min quarantine minimum, then
   dispose properly (salt-water bath is folklore — discharge via bulb + recycling point).
5. The battery never sits connected to the robot unattended; XT60 unplugged = parked.

### 4. Test cradle (used by P3 air-walk, P4 first-stand)
Purpose: robot suspended, belly supported, **all four feet in free air through the full leg
workspace**. Parametric design — exact cuts are TBD until material is chosen.

Driving dimensions (from `robot_params.yaml` body 258×117×85 mm; leg reach
femur 0.107 + tibia 0.100 ≈ **0.21 m**; coxa swing ±45°):
```
H_clear  : belly-to-floor  ≥ reach + margin        = 210 + 90  ≈ 300 mm
W_in     : inner width     ≥ 117 + 2×(210·sin45° + ~20 hip-y + 50 margin) ≈ 550 mm
L_in     : inner length    ≥ 258 + 2×120 swing margin                    ≈ 500 mm
H_bar    : strap bar height ≈ H_clear + body 85 + strap loop 150–250     ≈ 550–650 mm
Load     : hold ≥ 2× robot mass → design for ≥ 5 kg static (robot is 2.448 kg)
```
```
        ┌───────── top bar (H_bar) ─────────┐
        │   strap₁ (front)    strap₂ (rear) │     side view
        │      ╲                ╱           │
   post │     ┌─╲──────────────╱─┐          │ post      straps: 25–50 mm soft webbing,
        │     │  BARQ (belly up──┼──suppor- │           cam-buckle adjustable, routed
        │     │  on straps)      │  ted)    │           UNDER THE BELLY ONLY — never
        │     └──┬──┬──────┬──┬──┘          │           under legs/coxa (blocks the
        │       legs hang free              │           workspace, protocol §3 pinch
        │     ≥300 mm to floor              │           hazard at the tibia fold D-012)
   ─────┴───────────────────────────────────┴─────  floor (feet pads/sandbags on posts)
```
Build options (pick by what's obtainable): 25 mm PVC pipe + tees (fast, light, gussets via
cross-braces), or 2×2 lumber screwed + corner gussets, or a salvaged shelf frame. Two straps
(front/rear of belly), each releasable one-handed but only while the other still holds.
Cut list: **TBD — derive from the formulas after material choice; write it into this file.**

### 5. Label everything (do this BEFORE P2, it cannot be retrofitted honestly)
- Print the ID map first: `./st3215_diag.py plan` (matches `robot_params.yaml servos:`).
- Every servo gets its BARQ ID label at `set-id` time, on the servo body, not the cable
  (diagnostics README: "LABEL THE SERVO").
- Every bus lead and driver board: `BUS0-FL … BUS3-RR` at BOTH ends of every cable.
- Power leads: voltage + direction (`12V RAIL →`, `4S BATT`, `5V BEC`). Fuse holders: the
  fitted value, written on the holder.
- Convention: leg order FL/FR/RL/RR, joints coxa/femur/tibia (REP-103 frames,
  `docs/00_OVERVIEW.md`) — labels use these exact strings, no local dialects.

### 6. Instruments (mandatory → optional ladder)
| Rung | Instrument | What it unlocks (and which TBD it feeds) |
|---|---|---|
| MANDATORY | Digital multimeter (DC V, continuity, 10 A DC range) | chemistry verification (TBD-P0-4, with B3 charger), every rail/fuse/continuity check, crude ripple screen on AC-mV (TBD-P0-3 screen) |
| owned | INA260 breakout(s) in series | bench ammeter: run/stall currents TBD-P0-1/2; later the robot's own telemetry (P1-02) |
| optional 1 | DC clamp meter (must be DC/Hall type — AC-only clamps read 0) | current anywhere without breaking the circuit; safe stall tests; whole-robot draw in P4 |
| optional 2 | USB scope / entry DSO (≥ 25 MHz to see the 1 Mbaud servo bus; rails need far less) | real buck ripple number (TBD-P0-3 pass-bar), bus signal integrity when a servo "randomly" drops off in P2/P3 |
Buy rung 1/2 only when a concrete TBD demands them — the multimeter + INA260 cover all of
P0–P2.

## Acceptance gates
| Gate | Bar |
|---|---|
| **G0.5** | Bench dry-run: stations [A]–[D] physically arranged per §1; master switch kills the 12 V rail from operating position; multimeter reads the PSU within ±0.1 V of its display; ONE servo passes the full `diagnostics/README.md` per-servo procedure at 12.0 V (scan→status→sweep clean); every cable present on the bench is labeled per §5 |
| **G0.6** | Cradle: holds ≥ 5 kg static for 10 min with no contact between load and floor/frame and no joint slip; with the (unpowered) robot strapped in, full hand-swept leg workspace (coxa ±45°, femur ±90°, tibia 0→−2.2) touches nothing; one strap released → the other still holds the robot |

## Fallback ladder
- **A:** Bench as drawn (§1) + purpose-built cradle (§4).
- **B:** No room for the full layout → collapse [B]+[C] onto one half-table; cradle from
  salvage (sturdy stool/chair frame upside-down + the same two straps) — gates G0.5/G0.6
  unchanged, the gates define the bench, not the furniture.
- **C:** No cradle materials at all → robot belly-up on a rigid foam block taller than
  300 mm on the floor, strapped to the block; legs hang over the edges. Degraded (no
  belly-down air-walk posture) but P3's first servo-twitch tests can proceed; **P4
  first-stand stays blocked until G0.6 passes on a real cradle** — do not waive it.
- *Switch criteria:* A unbuildable within one session's effort → B immediately; B's salvage
  frame fails G0.6's load test twice → fix or C, and order/scrounge real material.

## Rollback
Everything here is furniture — rollback is dismantling. Two one-way doors to respect:
(1) servo ID labels: if IDs are ever reassigned, relabel in the SAME session
(`st3215_diag.py set-id`, then the sticker — an unlabeled re-ID'd servo poisons P2);
(2) never store the LiPo strapped into the robot or loose on the bench — it goes back to
zone [A] at session end (protocol §1 end-of-session ritual applies to hardware too).

## Artifacts to record (→ `docs/05_RESEARCH_LOG.md`)
- Photo of the bench passing G0.5 and the cradle passing G0.6 (with the load).
- The cradle's as-built dimensions vs the §4 formulas (update the TBD cut list HERE).
- The G0.5 single-servo run: servo ID, supply V, `status` readout, sweep observations.
- Any deviation from `diagnostics/README.md` wiring — and why (then reconcile the README).

## If this entire phase approach fails
(No bench space at all — e.g., work must happen on a kitchen table that clears every night.)
Go portable: one plastic crate = LiPo zone (tile + bag + sand bottle inside), one toolbox =
the entire [B]+[C] station (PSU brick, driver board, USB-UART, DMM, labels), cradle rung C
foam block stored flat. Setup/teardown ≤ 10 min, gates G0.5/G0.6 still apply at every
setup. What you may NOT do without ANY fixed arrangement: leave a charging battery, or run
multi-hour P2 sweep sessions unattended-adjacent. If even portable fails, P2+ hardware work
moves to wherever a bench exists (Krish's place, a lab, a makerspace) — the Jetson + repo
travel; the dev environment (P0-01) is location-independent by construction.
