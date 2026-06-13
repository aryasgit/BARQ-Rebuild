# P1-01 — Power Tree: 4S pack → fused branches → 12 V servo rail

> Phase P1 · verified against repo @ 0e5ddaf

## Objective
Build and verify the robot's electrical backbone: one 4S LiPo feeding (a) the Jetson directly,
(b) a mandatory high-current 12 V buck for the four servo driver boards, (c) a 5 V BEC for
accessories — every branch fused, master-switched, and proven by measurement before any other
P1 file is executed. Output: a powered bench where "every board talks" is even possible.

**Why the buck is non-negotiable:** 4S full charge ≈ 17.1 V (LiHV would be 17.4 V). The ST3215
servos and the Waveshare driver boards are rated to **12.6 V max**. Connecting 4S directly to
the servo rail destroys them. The Jetson Orin Nano (9–20 V input) is the only load that takes
4S directly.

## Prerequisites
- Parts on hand: 4S GENX Premium 5200 mAh pack (512 g), 12 servos, 4 driver boards, Jetson,
  Teensy, INA260s, BNO085. Purchases below are criteria-only (no SKUs) with ₹ ranges.
- Tools: digital multimeter (DMM) with 10 A range, screwdrivers, crimper/solder iron, heat
  shrink. Strongly recommended: DC clamp meter (₹2,000–4,500). Optional: oscilloscope, bench
  supply with current limit.
- LiPo bag, fire-safe charging spot, charger with selectable LiPo/LiHV profiles and balance
  port.
- Read `00_DOOMSDAY_PROTOCOL.md` §3 (safety absolutes). Master switch within arm's reach is a
  rule of THIS file's product: design it in, then obey it.

## Shopping criteria (no invented SKUs — buy to spec)
| Item | Criteria | ₹ range |
|---|---|---|
| 12 V buck (servo rail) | Input rated ≥ 22 V; output 12.0 V (adjustable trim-pot preferred); continuous current ≥ 1.5× worst-case rail draw (TBD-3 — expect 20–30 A class); synchronous; over-current/short protection; mountable heatsink | 20 A class ₹700–1,800; 30 A class ₹1,500–3,500 |
| 5 V BEC | Input ≥ 20 V; 5 V out, 3–5 A; short protection | ₹300–900 |
| Master switch | DC-rated ≥ 40 A (battery-disconnect / XT90-antispark-loop style acceptable) | ₹250–900 |
| Fuses + holders | Automotive blade fuses, inline holders, assorted 3–40 A | ₹300–600 |
| Wire | Silicone-insulated, gauges per table below | ₹500–1,200 |
| Connectors | XT60 (main), XT30 (branches), barrel plug 5.5×2.5 mm for Jetson | ₹300–800 |

## The tree (ASCII — build exactly this topology)
```
 4S LiPo GENX 5200 mAh  (floor 13.6 V .. full ~17.1 V; VERIFY chemistry before 1st charge)
  |  (pack lead, XT60-class — verify on pack)
 [F0] MAIN FUSE  (blade, value TBD-5 — derive after stall measurement)
  |
 (S0) MASTER SWITCH  >= 40 A DC, mounted within arm's reach of the work area
  |
  +----------------------+-----------------------------+
  |                      |                             |
 [F1] 5 A               [F2] TBD-6                    [F3] 3 A
  |                      |                             |
 JETSON ORIN NANO       12 V BUCK  (>= TBD-3 cont.)   5 V BEC (3-5 A)
 barrel 5.5x2.5 mm       | set to 12.0 V NO-LOAD       |
 center-positive         | BEFORE connecting boards    +-- accessories: lidar (P5),
 (verify on devkit)      |                                 fan, lighting. NOT the
  9-20 V input OK        |  12 V DISTRIBUTION (star)       Teensy (USB from Jetson),
  on 4S directly         |                                 NOT the BNO085 (3V3 from
                 +-------+-------+-------+                 Teensy).
                 |       |       |       |
                BD1     BD2     BD3     BD4    (Waveshare driver boards)
                FL      FR      RL      RR     (bus labels fixed in P1-03)
                3 servos each, daisy-chained 3-pin Feetech bus
```
- **Star vs daisy on the 12 V rail:** RECOMMEND STAR — four individual feeds from one
  distribution point (bus bar, solder splice block, or XT60 parallel board). A daisy chain
  puts board-1's connector and wire in series with all 4 boards' stall currents and makes one
  loose screw terminal a whole-robot brownout. Daisy is acceptable ONLY as fallback ladder B
  (below) if harness routing in the chassis defeats star wiring — then gauge the trunk for the
  FULL rail current and put the highest-load legs (front, per sim stance loading) nearest the
  source.
- Teensy is powered over USB from the Jetson (HARD FACT). Do NOT also feed Teensy VIN from
  the 5 V BEC; one supply path only. Grounds: see P1-03 common-ground rules.

## Wire gauge + connector table (per branch)
| Branch | Current (worst case) | Gauge (silicone) | Connector |
|---|---|---|---|
| Pack → F0 → S0 → split | whole robot (TBD-4) | 12 AWG (match pack lead) | XT60 |
| Split → buck input | servo tree / V_bat (TBD-6 calc) | 14 AWG | XT60 |
| Buck out → star point | full 12 V rail (TBD-3 basis) | 12–14 AWG, ≤ 15 cm | solder/bus bar |
| Star → each driver board | 3-servo stall (≈ 3 × TBD-1) | 16–18 AWG (decide after TBD-1) | XT30 → board screw terminal (verify board input) |
| Split → Jetson | ≤ 2 A typical, peak TBD-7 | 20 AWG | barrel 5.5×2.5 mm center+ (verify against devkit spec) |
| Split → BEC, BEC out | ≤ 3 A / ≤ 5 A | 20 AWG / 20–22 AWG | XT30 / JST or screw |
| All grounds | same as positive partner | same gauge | — |
Every connector polarity-keyed; every splice heat-shrunk; label both ends of every wire.

## TBD table (measure, never guess — each fill = research-log entry)
| ID | Value | Procedure that produces it |
|---|---|---|
| TBD-1 | single ST3215 stall current @ 12.0 V | Step 4 below (clamp/shunt stall bench) |
| TBD-2 | single servo loaded-holding current @ 12.0 V | Step 4 variant: hold against hand torque |
| TBD-3 | buck continuous rating to buy | worksheet below from TBD-1/2 |
| TBD-4 | whole-robot worst-case battery current | worksheet below + confirmed in P4 stand-up |
| TBD-5 | main fuse F0 value | 1.25 × TBD-4, rounded up to blade size, ≤ wire ampacity |
| TBD-6 | buck-input fuse F2 value | 1.25 × (12.0 × I_rail_worst)/(13.6 × 0.90) |
| TBD-7 | Jetson peak draw per nvpmodel mode | DMM/INA260 in series during `nvpmodel` mode sweep + stress load |
| TBD-8 | buck output ripple at load | Step 7 (DMM AC-mV; scope if available) |
| TBD-9 | pack chemistry (LiPo 4.20 / LiHV 4.35 V/cell) | Battery SOP step 1 |
| TBD-10 | battery sag: resting vs stance-load voltage delta | supervised discharge test, Step 9 / P4 stance |
| TBD-11 | whole-tree no-load current | Step 8 |

## Buck sizing worksheet (do the math with YOUR numbers)
1. **Measure TBD-1 (stall) and TBD-2 (loaded hold):** see Procedure step 4.
   Vendor materials put ST3215 stall in the ~2–3 A class at 12 V — treat that ONLY as a
   sanity bracket for your measurement, never as the design number (verify against the
   Waveshare/Feetech datasheet; your bench number wins).
2. **Worst-case rail current template** (stand-up is the worst realistic event: all four legs
   loaded simultaneously):
   `I_rail_worst = N_stall × TBD-1 + (12 − N_stall) × TBD-2`, with N_stall = 4 (one joint per
   leg near stall during stand-up) as the default scenario. Record the scenario you chose.
3. **Buck rating:** `TBD-3 = round_up(I_rail_worst × 1.5)` (≥ 50 % headroom, contract
   requirement). If TBD-3 > 30 A, prefer fallback B (two bucks) over exotic single units.
4. **Battery-side check:** `I_bat_from_servos = 12.0 × I_rail_worst / (V_bat_floor 13.6 × η 0.90)`.
   `TBD-4 = I_bat_from_servos + I_jetson(TBD-7) + I_bec(≤1 A)`. Confirm pack C-rating covers it
   (5.2 Ah × pack's printed C ≥ TBD-4 — read C off the label, don't assume).

## Procedure (numbered, with expected readings)
1. **Battery SOP first** (below) — gate G1.0 before the pack powers anything.
2. **Bench the buck alone.** Input from pack (through F0+S0) or bench supply at ~16 V. No
   load. Trim output to **12.00 V** (expect adjustable range; set with DMM, ±0.05 V).
   Expected: stable 12.00 V, input current < 50 mA idle.
3. **Load the buck incrementally** with a dummy load if available (e.g., 12 V/50 W bulbs or
   power resistors): at each step check output ≥ 11.7 V and buck temperature by touch/IR
   (< 85 °C). Expected: droop < 0.3 V to its rated current.
4. **Single-servo stall bench (produces TBD-1/TBD-2).** One ST3215 on one driver board, fed
   from the buck at 12.0 V. Current measurement: DC clamp meter around the +12 V wire, OR an
   INA260 in series (one servo stays under its 15 A ceiling), OR DMM 10 A range (keep stall
   < 5 s — DMM shunts heat). Command a move with `diagnostics/st3215_diag.py move`, then
   gently stall the horn by hand through a rag/lever (torque limit at default). Read peak
   current. Note: the diagnostics README's "7.4 V class supply" note is the conservative
   bench-tool default — for THIS measurement you must be at 12.0 V, the real rail voltage,
   because stall current scales with voltage. Repeat 3×, keep the max. Also record
   `st3215_diag.py status` voltage reading during stall (sag visibility at the servo).
5. **Complete the worksheet** → buy/select buck (TBD-3) and fuses (TBD-5/6). Until fuses are
   derived, bench work proceeds only behind the bench supply's current limit or a provisional
   10 A main fuse with only ONE driver board connected.
6. **Assemble the tree** per the ASCII diagram. Run the pre-power checklist (below) — every
   line, every time the harness changes.
7. **Ripple/dropout sanity (TBD-8).** DMM on AC mV across the buck output while one servo
   sweeps (`st3215_diag.py sweep`): expect < 150 mV AC indicated (DMM AC-mV on a switcher is
   crude but catches gross instability). Scope if available: < 100 mVpp ripple, no dips below
   11.7 V during stall onset. Min DC during stall: ≥ 11.7 V at the BOARD terminals (measures
   harness drop too, not just buck regulation).
8. **Whole-tree no-load current (TBD-11).** Everything connected, servos torque-off, Jetson
   idle at its default nvpmodel mode, Teensy running v0 firmware. Measure battery current.
   Expected: dominated by the Jetson (order 0.4–1 A at 4S); buck idle + boards + BEC add
   tens of mA. Record the number; it is the future "is something wrong at power-on" baseline.
9. **Supervised brownout rehearsal (G1.3).** With a partially discharged pack (≈ 14.2 V
   resting), run servos under continuous sweep load and watch battery voltage (DMM on the
   pack, and INA260 telemetry once P1-02 is wired): confirm you can OBSERVE the ladder
   14.0 → 13.8 → 13.6 V and that at 13.6 V you stop work (manual today; firmware enforces in
   P3/P4). Record resting-vs-load delta (TBD-10).

## Brownout policy (defaults to confirm in P3/P4)
| Threshold | Value | Action |
|---|---|---|
| Warn | 14.0 V (3.50 V/cell) | operator warning in telemetry |
| Power alert | 13.8 V (3.45 V/cell) | STATE fault **bit2** set (protocol 06_PROTOCOL.md) |
| Floor | 13.6 V (3.40 V/cell, team decision) | controlled sit-down, torque off |
**Why sag matters:** a pack reading 14.4 V resting can dip below 13.6 V during stance/stall
transients (internal resistance × current). Thresholds act on the VOLTAGE UNDER LOAD the
firmware sees, so a pack near the warn level will flap across thresholds with gait phase.
TBD-10 quantifies the delta; P4 may add hysteresis/filtering (e.g., 1 s rolling minimum)
using that number. Until then: thresholds above are evaluated on the battery-side measurement
point chosen in P1-02 (the 12 V rail is regulated and CANNOT see pack sag — battery-side
sensing is mandatory; see P1-02 placement).

## Battery SOP
1. **Chemistry verification (TBD-9, BEFORE first charge):** (a) read the pack label — any
   "LiHV/High-Voltage/4.35 V" marking? (b) count balance-lead pins = 5 for 4S; (c) measure
   per-cell voltages on the balance lead with the DMM — as-shipped storage charge reads
   ~3.75–3.90 V/cell for either chemistry, so the label is the authority; (d) if the label is
   ambiguous or absent, CHARGE AS LiPo 4.20 V/cell — undercharging LiHV is safe, overcharging
   LiPo is a fire. Set the charger profile accordingly and write the decision in the log.
2. **Charging:** balance-charge every time; ≤ 1C (5.2 A), first charge at ≤ 0.5C while
   watching per-cell convergence; supervised, in the LiPo bag, on a non-flammable surface.
3. **Cell-balance cadence:** record per-cell deltas after every charge; investigate > 30 mV,
   retire/quarantine > 60 mV persistent (verify thresholds against charger manual).
4. **Storage/transport:** 3.80–3.85 V/cell if idle > 3 days (protocol §3 rule); transport
   only at storage charge in the bag; never charge/store on the robot.
5. **Damage:** puffy/hot/smelly → outdoors, 30 min, salt-water decommission per local norms.

## Pre-power checklist (every harness change)
- [ ] Pack DISCONNECTED; master switch OFF; fuses OUT.
- [ ] Continuity: battery+ node to each branch + (with fuses inserted), no + to − short
      (DMM beeper) at pack connector, at buck in, at buck out, at each board terminal.
- [ ] Polarity at EVERY connector verified against the diagram (red/+ marking continuity).
- [ ] Buck output trimmed to 12.0 V no-load BEFORE driver boards are connected (a mis-set
      buck at >12.6 V kills all four boards at once).
- [ ] BEC output verified 5.0 V ± 0.25 no-load before accessories.
- [ ] Master switch within arm's reach; fire-safe surface; LiPo bag staged.
- [ ] First power-up of any new configuration: fuses in, switch ON, hand stays on switch
      10 s, watch/smell/listen; then measure all rail voltages before connecting loads.

## Acceptance gates
- **G1.0** — pack chemistry verified (TBD-9 logged), first balance charge completed with
  per-cell delta ≤ 30 mV; full-charge voltage matches chemistry (16.8 V LiPo / 17.4 V LiHV
  ± 0.1).
- **G1.1** — buck holds **12.0 V ± 0.3 V at the driver-board terminals** during a
  single-servo hand-stall (step 4 setup), no thermal shutdown, temp < 85 °C.
- **G1.2** — whole-tree no-load current (TBD-11) measured, stable for 5 min, and explainable
  to within 20 % as Jetson-idle + quiescent draws (no mystery hundreds of mA).
- **G1.3** — on a supervised discharge, the operator observes 14.0/13.8/13.6 V crossings on
  the battery-side measurement and stops at 13.6 V; resting-vs-load sag (TBD-10) recorded.

## Fallback ladder
- **A (default):** single 12 V buck (TBD-3 rating), star distribution.
  *Switch to B if:* the buck fails G1.1 twice (after checking harness drop and trim), OR
  sustained-load temp > 85 °C, OR TBD-3 computes > 30 A making single units exotic/costly.
- **B:** TWO medium bucks (each ≥ 0.75 × TBD-3), one per pair of driver boards
  (FL+FR / RL+RR), each pair fused separately. Pairs match the INA260 placement option in
  P1-02. *Switch to C if:* B still browns out under stall tests, or procurement of suitable
  bucks fails within budget after 2 attempts.
- **C:** tethered development — servo rail from a mains 12 V supply ≥ TBD-3 (bench PSU or
  repurposed server PSU, ₹1,500–4,000) while battery powers Jetson only. Robot is not
  field-capable on C; P4 untethered gates are blocked, everything else proceeds.

## Rollback
Any smoke/smell/heat or failed gate: switch OFF → pack out → fuses out → return to the
known-good bench configuration (driver board on USB + bench/2S-class supply per
`diagnostics/README.md`) and re-prove a single servo there before re-attempting the tree.
Log what was connected at the time of failure (photo of the harness).

## Artifacts → docs/05_RESEARCH_LOG.md
TBD-1..11 values with date/instrument; buck make/rating as bought + trim setting; fuse map;
photo of the assembled tree + labels; G1.0–G1.3 pass evidence (numbers, not adjectives);
any overridden decision (e.g., daisy instead of star, and why).

## If this entire phase approach fails
If 4S + 12 V buck cannot be made reliable (repeated brownouts/failures through ladders A–C):
re-architect to a **3S pack feeding the servo rail directly** (3S full = 12.6 V — exactly the
servo/board ceiling, VERIFY board tolerance at full charge; floor 3.4 V/cell = 10.2 V keeps
the Jetson inside but with thin margin — re-derive Jetson supply, possibly a boost or
separate pack). Costs: new pack (₹2,500–4,500), lower nominal torque at 11.1 V, redone
worksheets; benefit: removes the buck as a single point of failure. This is an architecture
change — write it up in docs/02_DECISIONS.md before spending money.
