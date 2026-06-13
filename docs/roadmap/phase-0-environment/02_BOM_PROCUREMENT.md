# P0-02 — BOM: what we own, what to buy, and how to choose it

> Phase P0 · verified against repo @ 0e5ddaf

## Objective
A complete parts picture before any hardware phase starts: an OWNED inventory (verify it
physically — owned ≠ findable), and a TO-BUY list specified by **selection criteria, not
SKUs** (vendors and prices drift; criteria don't). Every TO-BUY row is tagged with the first
phase it blocks, so a tight budget can be spent in dependency order. Prices are ranges in ₹ —
**verify at order time**; never treat a range here as a quote.

Power architecture this BOM serves (decided 2026-06-12, see `docs/roadmap/README.md`):
```
4S LiPo 5200 mAh (≈16.8–17.1 V full, floor 13.6 V)
 ├─ [fuse] ──► Jetson Orin Nano (9–20 V input — DIRECT from 4S)
 └─ [fuse] ──► high-current 12 V BUCK ──► servo rail (4× driver boards, 12 ST3215)
                                   └─► (option) 5 V BEC ──► lidar/logic   [or BEC from 4S]
Teensy: USB power from the Jetson. 4S > 12.6 V servo/driver max → the buck is NOT optional.
```

## Prerequisites
- P0-01 gates green (a working dev environment proves the repo claims this BOM rests on).
- `diagnostics/README.md` read once (the bench rows below exist to serve that procedure).
- Budget reality: lidar ceiling is **₹24,000** (Q-015); everything else is sized to be
  individually small but it adds up — total the sheet before ordering.

## Procedure
1. Physically locate and check off every OWNED row; quarantine anything damaged.
2. Fill the "verify first" rows (battery chemistry label, driver-board USB capability) —
   they change what you buy.
3. Order TO-BUY rows tagged **P1** first, then P2; defer the lidar row to P5 by design.
4. Log every order (vendor, price, date) in `docs/05_RESEARCH_LOG.md`; tick G0.4.

### Table 1 — OWNED (verify physically, don't trust this list)
| # | Item | Qty | Notes / where it's load-bearing |
|---|---|---|---|
| O1 | Waveshare ST3215 serial bus servo, 30 kg·cm @ 12 V (Feetech STS protocol, 1 Mbaud) | **12** | The robot. Bench tool: `diagnostics/st3215_diag.py`. Stall/run currents are **TBD — measure, don't trust the listing** (TBD-P0-1/2) |
| O2 | Waveshare serial bus servo driver board | **4** | One per leg, 3 servos chained per bus (matches Teensy's 4 UARTs; `diagnostics/README.md`). **Verify-first:** does the owned variant expose USB directly? → decides row B9 |
| O3 | Teensy 4.1 | 1 | Firmware `barq_firmware/` (PlatformIO on the host). Powered via USB from Jetson |
| O4 | Jetson Orin Nano (+ PSU, NVMe/SD) | 1 | JetPack 6 / L4T r36.4.x (P0-01). 9–20 V input → takes 4S directly, fused |
| O5 | BNO085 IMU | 1 | Stage-3 `/imu/data` source (docs/06_PROTOCOL.md). Older docs say INA226/“BNO085+INA226” — INA260 is what we own (roadmap README correction) |
| O6 | INA260 power monitor (integrated 2 mΩ shunt, ±15 A ceiling per unit) | several | P1-02 monitoring design; also the bench current instrument until a clamp meter exists |
| O7 | 4S LiPo GENX Premium 5200 mAh, 512 g | 1 | Already counted in the 1420 g body mass and pending CoM (Q-014). **Chemistry UNVERIFIED** (LiPo 4.20 vs LiHV 4.35 V/cell) — TBD-P0-4 BEFORE first charge |
| O8 | Printed frame (body 258×117×85 mm + legs) | 1 set | `robot_params.yaml` is the dimensional truth; D-012 tibia fold [−2.2, 0] is a pinch hazard by design |

### Table 2 — TO BUY (selection criteria, not SKUs)
| # | Item | Selection criteria | ₹ range (verify at order time) | Blocks |
|---|---|---|---|---|
| B1 | **High-current 12 V buck** (servo rail) | Output fixed/adjustable to 12.0 V (≤12.6 V hard max — driver-board limit); input ≥ 18 V (4S full); continuous current per the sizing template below (**finish TBD-P0-1/2 before final selection**, order after bench measurement or buy adjustable & generously oversized); synchronous design preferred (efficiency → heat); screw terminals or solderable pads for 14 AWG; exposed output caps you can scope (ripple eval below) | 800–2,500 | **P1** |
| B2 | **5 V BEC** (logic/lidar rail) | ≥ 2 A continuous (lidar spin-up headroom — research doc §2: never lidar from a Jetson USB port), 5.0–5.2 V, input tolerant of 4S OR fed from the 12 V rail; low ripple (lidar wants ≤ 50 mV class) | 300–800 | P1 (logic) / **P5** (lidar) |
| B3 | **Balance charger** with per-chemistry profiles | Explicit selectable LiPo (4.20 V/cell) AND LiHV (4.35 V/cell) modes; per-cell balance display (this is the chemistry-verification instrument, TBD-P0-4); storage-charge mode; ≥ 3 A charge | 1,800–4,500 | **P1** (before FIRST charge) |
| B4 | LiPo safety bags (≥ 2) + dry sand bucket | Bag sized for a 4S 5200; second bag for charging vs storage | 400–900 | **P1** |
| B5 | Connectors: XT60 pairs (battery main, ~30 A class) + XT30 pairs (sub-rails, ~15 A class) | Genuine Amass-type preferred (clones overheat); at least 4× XT60, 6× XT30 pairs + heatshrink assortment | 500–1,200 | **P1** |
| B6 | Silicone wire, 14 AWG and 16 AWG (12 V rail), 20–22 AWG (logic) | See gauge table below; silicone insulation (iron-proof, flexible); red+black, ~2 m each heavy gauge | 500–1,200 | **P1** |
| B7 | Inline fuse holders ×3 + blade-fuse assortment (5–40 A) | Holders rated ≥ 40 A, 14 AWG leads; **values are TBD-P0-5 — from measurement, not guessing**; buy the assortment box so any measured value is coverable same-day | 300–700 | **P1** |
| B8 | **Master switch** (battery main) + e-stop per strategy ladder below | Continuous rating ≥ the fused battery-main current with margin; DC-rated (AC ratings lie at DC — arc quench); mountable within arm's reach (protocol §3 torque-off rule) | 300–1,000 | **P1** |
| B9 | USB-UART adapter, CP2102 or CH340 class (servo bench) | 3.3 V TTL, ≥ 1 Mbaud verified (CP2102 ok; cheapest CH340 boards top out lower — check the listing); **only needed if O2 boards lack onboard USB (verify first)**. Host prep: `brltty` removed + dialout group (P0-01 step 2) | 150–400 | **P2** (bench calib; P1 bench comms) |
| B10 | Bench power: see ladder below | Plan A: adjustable bench PSU 0–20 V ≥ 10 A, current-limit knob, displayed V/A | A: 4,000–9,000 | **P1/P2** (all bench work) |
| B11 | **Lidar — DECIDED-PENDING-PURCHASE, defer order to P5** | Primary: **LDROBOT STL-27L** (~45 g, 25 m, fits sim model already built) ≈ ₹13k well under the ₹24k ceiling; fallback: LD19/D500 class (~47 g, 12 m) if STL-27L unavailable/over-ceiling. Full analysis: `docs/research/2026-06-11-lidar-selection.md`, Q-015 | 7,000–24,000 (ceiling) | **P5** only — do NOT buy in P0 |
| B12 | Spares: servo horns + horn screws, M2/M2.5/M3 screw assortment, 3-pin JST-PH style bus leads (servo daisy-chain), zip ties, label maker tape | Match the ST3215 horn spline (verify against an owned servo before ordering); bus leads: buy ≥ 6 (they fail first) | 500–1,500 | nice-to-have (saves P2/P3 days) |
| B13 | Spare ST3215 servo ×1 (budget permitting) | Identical model/voltage variant to O1 (30 kg·cm @ 12 V) | 2,000–3,500 | none (insurance — the swap-test rung in protocol §4) |

### 12 V buck sizing template (fill with MEASURED numbers, then buy)
Symbols: `I_stall` = one servo's stall current at 12 V (TBD-P0-2); `I_run` = one servo's
typical walking current (TBD-P0-1); both measured with an INA260 on the bench, not read off
a listing.

```
Absolute worst case (all-stall):      I_abs  = 12 × I_stall          # fault state, the FUSE's job
Engineering case (design load):       I_eng  = 4 × I_stall + 8 × I_run
   # rationale: trot loads ~2 legs hard; 4 simultaneous near-stall joints is a generous
   # jam/transient envelope; the remaining 8 tick over at run current
Buck continuous rating:               I_buck ≥ 1.5 × I_eng           # ≥50% headroom, prompt-cooled
Sanity checks before ordering:
   - I_buck ≥ measured whole-robot walking peak × 1.5 (once P4 data exists, re-verify)
   - servo-rail fuse value (TBD-P0-5) < buck's continuous rating  → fuse blows before buck cooks
   - buck input current at 13.6 V ≈ I_out × 12/13.6 ÷ efficiency — size the battery-side wiring/fuse for THIS
```
Worked example **with placeholder numbers — do not order off these**: if bench gives
I_stall = 3 A, I_run = 0.4 A → I_eng = 12 + 3.2 = 15.2 A → buy ≥ 23 A continuous. If nothing
in budget reaches the template number, rung B of the bench-power ladder keeps P1/P2 moving
while you save up — single-servo bench work needs only a few amps.

**Buck ripple evaluation (acceptance, on arrival):** at a representative load (one servo
sweeping under `st3215_diag.py sweep`, more if available): multimeter on AC-mV across the
output caps as a crude screen, USB scope if owned (instrument ladder, `03_BENCH_SETUP.md`)
for the real picture. Symptoms of a bad buck: servo resets/bus dropouts on direction
reversals, audible whine, output sag > a few hundred mV under step load. Numeric pass bar is
TBD-P0-3 — set it from the first scope trace of a known-good run and write it HERE.

### Wire gauge by continuous current (silicone, short runs < 50 cm)
| AWG | Conservative continuous | Use on BARQ |
|---|---|---|
| 20–22 | ≤ 3 A | logic, 5 V sensor leads |
| 18 | ≤ 8 A | single-servo bench leads |
| 16 | ~10–15 A | per-bus 12 V feeds (3 servos) |
| 14 | ~20–25 A | battery main, buck input/output trunk |
Verify against the actual wire's manufacturer rating; at these lengths voltage drop
(ΔV = I × R_per_m × 2L) hurts servos before heating does — compute it for the trunk run.

### E-stop strategy ladder (decide in P1, buy the parts now)
- **A — battery-main mechanical switch** (B8) within arm's reach: kills everything. Simple,
  total, but also kills the Jetson (unclean shutdown).
- **B — servo-rail-only switch/loop key** (second B8-class switch or an XT60 loop key
  between buck and rail): kills torque, keeps the Jetson alive — the nicer mid-experiment
  stop. RECOMMENDED as the standard bench e-stop, with A as the fire-stop.
- **C — software stop + firmware deadman** (200 ms, fault bit3): already exists, never
  disabled (protocol §3) — but software is NEVER the only stop. C alone is not a strategy.
Switch criteria: start with A only (P1 bring-up); add B before the first all-12 torque-on
(P2 exit); C is always-on background.

### Bench power option ladder (row B10)
- **A — adjustable bench PSU, 0–20 V ≥ 10 A** with current limit: the right answer. Current
  limit knob is the single best servo-saver during first power-ups (set 12.0 V, limit ~1 A
  for first contact, raise as trust grows).
- **B — fixed 12 V ≥ 5 A brick** (laptop-class/LED supply): fine for single-servo bench and
  3-servo bus work; no current limiting — add the inline fuse (B7) and an INA260 in series.
  Switch to B if A is out of budget this month; bench work must not stall.
- **C — the 4S battery through the 12 V buck (B1)**: works, but a 5200 mAh 4S can dump
  hundreds of amps into a mistake — ONLY with: inline fuse fitted, LiPo rules
  (`03_BENCH_SETUP.md` §LiPo), never unattended, master switch in reach. Use C only when A
  and B are both unavailable AND the buck has already passed its ripple check.

## Acceptance gates
| Gate | Bar |
|---|---|
| **G0.4** | Every TO-BUY row tagged **P1** is status ORDERED or IN-HAND, logged (vendor/price/date) in `docs/05_RESEARCH_LOG.md`; both verify-first rows (battery chemistry label read; driver-board USB capability) answered; the TBD table below copied into the log with owners |

## Fallback ladder (procurement itself)
- **A:** Order per the criteria above from any reputable Indian electronics/hobby vendor;
  buy the adjustable/oversized option whenever the measured TBD isn't in yet.
- **B:** Item unavailable in-country / over budget → substitute by criteria (the tables
  carry no SKUs precisely so any substitute meeting the column is valid); for the buck,
  two paralleled smaller bucks per 2-bus half-rail is an acceptable B (document the split
  and fuse each half).
- **C:** Budget exhausted → buy only: bench supply rung B, B5–B7 (connectors/wire/fuses),
  B9 if needed. That subset unblocks ALL of P2 calibration on a tethered bench; battery
  chain (B1–B4, B8) waits for the next budget cycle, P4 untethered slips accordingly.
- *Switch criteria:* two vendors fail you on an item or the quote exceeds the top of the
  range by >50% → rung B for that item; total sheet > available budget → rung C triage.

## Rollback
Procurement rollback = vendor returns where possible; otherwise: anything bought oversized
(buck, PSU, wire gauge) is never wasted — oversizing is the default here by design. The only
unrecoverable mistake this file can produce is charging the battery with the wrong chemistry
profile — which is why TBD-P0-4 + the B3 criteria gate the FIRST charge, not the purchase.

## Artifacts to record (→ `docs/05_RESEARCH_LOG.md`)
- The filled OWNED checklist (anything missing/damaged is a finding).
- Order log: row #, vendor, exact item name, price paid, date, expected arrival.
- The two verify-first answers (battery label photo; driver-board USB yes/no).
- This TBD table, copied with status:

| TBD | Value | Measurement that produces it | Unblocks |
|---|---|---|---|
| TBD-P0-1 | `I_run` per servo @ 12 V (idle + loaded sweep) | INA260 in series during `st3215_diag.py sweep` (P1-01 bench) | buck + fuse final sizing |
| TBD-P0-2 | `I_stall` per servo @ 12 V | ≤ 2 s stall against a held horn, INA260 peak, current-limited PSU (P1-01) | buck template, fuse values |
| TBD-P0-3 | buck output ripple pass-bar (mV pp @ load) | first scope/DMM-AC trace of a known-good sweep on the chosen buck | B1 acceptance |
| TBD-P0-4 | battery chemistry (LiPo 4.20 / LiHV 4.35) | pack label + per-cell voltage on the B3 charger display at arrival state | FIRST charge |
| TBD-P0-5 | fuse values (battery main / servo rail / Jetson) | measured peaks (TBD-P0-1/2 + Jetson draw) × ~1.5, next standard size up | P1 power tree |
| TBD-P0-6 | driver-board USB capability (O2 variant) | inspect the owned board; plug in, `ls /dev/ttyACM* /dev/ttyUSB*` | whether B9 is bought |

## If this entire phase approach fails
(No budget at all, or supply chain dead for power parts.) The robot can still advance: P2
servo calibration runs on ANY clean 12 V source ≥ 2 A you can scrounge (rung B brick, even a
salvaged PSU) + the owned driver boards; P3 firmware integration needs only the Teensy + one
bus tethered; P6 RL env work needs nothing physical. Re-scope P4 to "tethered stand/walk on
bench supply" and park the battery chain entirely — an extension-cord robot that walks is
worth more than a complete BOM that never ships. Revisit this file when money exists; the
criteria don't expire, the prices do.
