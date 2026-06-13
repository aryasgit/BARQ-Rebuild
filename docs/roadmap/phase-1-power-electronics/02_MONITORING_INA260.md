# P1-02 — Power Monitoring with INA260 (±15 A/unit ceiling)

> Phase P1 · verified against repo @ 0e5ddaf

## Objective
Design and bench-validate the current/voltage monitoring that feeds the protocol's
`vbus` (mV) and `current` (mA) STATE fields and the brownout ladder of P1-01 — using the
**owned INA260s** (integrated 2 mΩ shunt, **±15 A continuous ceiling per unit**), with the
INA226 + external shunt as the documented fallback when any single monitored path must
exceed 15 A. P3 fills the firmware stub `power_read()` (`barq_firmware/src/loop_core.cpp`);
this file delivers the hardware, addresses, and thresholds that stub will use.

**Doc corrections, explicit:** `docs/06_PROTOCOL.md` labels the vbus/current fields
"(INA226)" — that is the older plan. The fields are chip-agnostic; the owned chip is the
INA260 (roadmap README "hard truths"). Also note the current `power_read()` stub returns
7400 mV — a 2S-era placeholder; never "validate" telemetry against it.

## Prerequisites
- P1-01 executed at least through step 5 (TBD-1 stall current measured — the placement
  decision below needs it; buck + tree may still be in assembly).
- Teensy 4.1 on the bench (USB to host), PlatformIO working on the host (P0), Jetson serial
  prep done if benching via Jetson (`sudo apt remove brltty`, user in `dialout`).
- INA260 breakout boards (owned), hookup wire, 1× known load (power resistor: 10 Ω ≥ 25 W
  for sustained ≈1.2 A, or 4.7 Ω ONLY if ≥ 50 W rated — 12 V across 4.7 Ω is ~31 W;
  ₹100–400 — or one servo held at stall), DMM.

## Placement (decision table driven by measured numbers)
The INA260 measures its OWN bus-pin voltage, so WHERE it sits decides what `vbus` means.
The brownout ladder (14.0/13.8/13.6 V) is defined on BATTERY voltage — the regulated 12 V
rail cannot show pack sag. Therefore:

| # | Location | What it gives | Use when |
|---|---|---|---|
| M0 | **Battery side, in series with the buck INPUT** (after F2) | pack vbus for the brownout ladder + total servo-tree input current (scaled by 12/V_bat·η ≈ 0.8× of rail current — less likely to hit 15 A) | **ALWAYS — this is the recommended primary unit.** |
| M1, M2 | 12 V rail, one per PAIR of driver boards (FL+FR, RL+RR) | per-pair servo current; localizes a fault to half the robot | required if `6 × TBD-1 > 15 A` would overload a single rail monitor; pairs ≤ 15 A must hold: `6 × TBD-1 ≤ 15 A` |
| M3 | Jetson branch | compute draw (TBD-7 from P1-01) | optional, if a third/fourth unit is spare |

Decision procedure: compute `I_rail_worst` (P1-01 worksheet). If `I_rail_worst ≤ 15 A`, a
single 12 V-rail unit is also legal — but M0 is still required for vbus, so the minimal
correct build is **M0 alone** (current = servo tree total at battery side), and the
recommended build is **M0 + M1 + M2** if three units are on hand. If any single monitored
path exceeds 15 A → INA226 fallback (below) for that path. Record the chosen set as a
decision entry.

**Protocol field mapping (defaults for P3 to confirm):** `vbus` ← M0 bus voltage (battery,
mV). `current` ← M0 current (mA, battery-side servo-tree draw; if M1/M2 fitted, firmware MAY
instead report M1+M2 sum — record which, the field is single-valued). Per-pair values and
the Jetson branch ride in a future STATE rev or debug channel; do not widen the frame now.

## Wiring (I2C to the Teensy)
ASCII (M0 shown; M1/M2 identical except address straps):
```
  BATTERY+ --[F2]--> INA260 VIN+  >>>  VIN- --> BUCK VIN+        (current path, 12 AWG-class
                       |  (M0 in series with buck input)          lugs/terminals - keep stubs
  BATTERY- ----------------------------+--> BUCK VIN-             short, solder or screw)
                                       |
  Teensy 4.1                           |
   3V3  ------------- VS (INA260 supply: 3.3 V)
   GND  ----+-------- GND -------------+   (signal ground ties to power ground AT ONE POINT)
   18 SDA --+--[4.7k pull-up to 3V3]-- SDA
   19 SCL --+--[4.7k pull-up to 3V3]-- SCL
   (optional) GPIO  <---------------- ALERT (open-drain, needs pull-up; pick a free pin, e.g. 4)
   A1/A0 straps ----- per address table below
```
- Bus: **Teensy `Wire` = SDA pin 18, SCL pin 19** (public Teensy 4.1 mapping — verify
  against the PJRC pinout card). One 4.7 kΩ pull-up pair per bus to 3.3 V total (if each
  breakout carries its own pull-ups, confirm the parallel total stays ≥ ~1.5 kΩ; remove
  extras otherwise).
- Power the INA260 VS from Teensy 3V3 so logic levels match (chip tolerates more; the Teensy
  does not — 3.3 V only).
- Keep the BNO085 on a DIFFERENT I2C port (Wire1, per P1-04) so an IMU lockup can't stall
  power telemetry and vice versa. Pin plan consistency: P1-03 reserves serials so that
  Wire (18/19) and Wire1 (16/17) stay free.
- The current path (VIN+/VIN−) is POWER wiring: same gauge as the branch it interrupts
  (P1-01 table); the I2C side is signal wiring (any 24–28 AWG, twisted with GND, < 30 cm).

## Address straps (public INA260 table — verify against TI datasheet §7.5)
7-bit address = function of A1, A0 pin connections:
| A1 \ A0 | GND | VS | SDA | SCL |
|---|---|---|---|---|
| **GND** | 0x40 | 0x41 | 0x42 | 0x43 |
| **VS**  | 0x44 | 0x45 | 0x46 | 0x47 |
| **SDA** | 0x48 | 0x49 | **0x4A** | **0x4B** |
| **SCL** | 0x4C | 0x4D | 0x4E | 0x4F |
Assign: M0=0x40 (A1=GND,A0=GND), M1=0x41 (A0=VS), M2=0x44 (A1=VS), M3=0x45. **Never use
0x4A/0x4B** (bold above) anywhere in the robot — those are the BNO085's addresses; even on
separate buses, keeping them globally unique prevents a mis-plug from becoming a debugging
nightmare. Label each unit with its address.

## Register map summary (public — verify against TI INA260 datasheet §7.6)
| Reg | Name | Notes / conversion to protocol fields |
|---|---|---|
| 0x00 | CONFIG | avg count, conversion times, mode; power-on default 0x6127 (continuous) |
| 0x01 | CURRENT | two's complement, **LSB = 1.25 mA** → `current_mA = reg × 1.25` (int16 field holds ±32 A: fits) |
| 0x02 | BUS_VOLTAGE | **LSB = 1.25 mV**, always positive → `vbus_mV = reg × 1.25` (uint16 ceiling 65.5 V: fits 4S) |
| 0x03 | POWER | LSB = 10 mW (telemetry nicety, not in the STATE frame) |
| 0x06 | MASK/ENABLE | alert source select (e.g., bus under-voltage), latch, conversion-ready |
| 0x07 | ALERT_LIMIT | threshold in the units of the selected source (1.25 mV/LSB for bus UV) |
| 0xFE/0xFF | MFG/DIE ID | 0x5449 ('TI') — use as the presence self-test |
Config recommendation for 100 Hz STATE: averaging 16, conversion time 1.1 ms, continuous
mode — effective update ≈ 1/(16 × 1.1 ms × 2) ≈ 28 Hz per channel-pair; the 100 Hz loop
re-reports the latest sample, which is fine for power (it is brownout protection, not a
control signal). Tighten only if P4 wants faster sag detection.

## Alert strategy (options ladder)
- **A (default): polled thresholds in firmware.** P3's `power_read()` reads CURRENT +
  BUS_VOLTAGE each loop; threshold ladder (14.0/13.8/13.6 V) implemented in C++ on the M0
  vbus value; fault **bit2** set at ≤ 13.8 V. Simple, no extra wiring, one source of truth.
- **B: ALERT pin as hardware backup.** MASK/ENABLE = bus under-voltage, ALERT_LIMIT =
  13.8 V (0x2B20 ≈ 11040 × 1.25 mV — recompute, verify), latch enabled; ALERT (open-drain)
  → Teensy GPIO with pull-up; firmware treats pin-low as bit2 regardless of polling.
  *Switch A→B when:* polled loop jitter or I2C contention ever causes a missed/late (>50 ms)
  threshold crossing in P3 HIL tests.
- **C: both + watchdog semantics** (pin latched until read): adopt if P4 logs show transient
  sags shorter than the polling interval mattering (TBD-10 says how short sags get).

## INA226 + external shunt fallback (any path > 15 A through one monitor)
The INA226 (older docs' chip) measures across an EXTERNAL shunt — size the shunt, not the
chip (bus pin still reads the rail voltage, 0–36 V):
1. Full-scale shunt voltage = ±81.92 mV (public, verify datasheet). Choose
   `R_shunt ≤ 81.92 mV / I_max_path`. Example template: I_max 30 A → R ≤ 2.73 mΩ → pick
   **2 mΩ** (₹100–300 for a 3 W+ bolt-down/PCB shunt).
2. Power: `P = I² × R` (30 A → 1.8 W on 2 mΩ) → rating ≥ 3 W, prefer 5 W + airflow.
3. Calibration: `Current_LSB = I_max / 2^15` (30 A → 0.916 mA, round to 1 mA);
   `CAL = 0.00512 / (Current_LSB × R_shunt)` (1 mA, 2 mΩ → 2560). Verify formula against TI
   INA226 datasheet §7.5 before flashing constants.
4. Same I2C bus/pull-up/address discipline (INA226 shares the 0x40–0x4F strap scheme —
   verify its table; keep clear of 0x4A/0x4B).
Kelvin-connect the shunt (sense wires from the shunt pads, not the lugs) or the 2 mΩ math
drowns in contact resistance.

## Procedure (bench validation)
1. Strap M0 to 0x40, wire per diagram on `Wire` (18/19), INA260 in series with a KNOWN load
   on the bench (NOT yet the buck input): 12.0 V buck output → INA260 → power resistor
   (10 Ω ⇒ expect ≈ 1.2 A; 4.7 Ω ⇒ ≈ 2.55 A but ~31 W, keep it brief unless ≥ 50 W rated)
   or one servo held at stall.
2. I2C presence: scan from Teensy (any I2C scanner sketch; P3 doc carries the real driver).
   Expected: device ACKs at 0x40; MFG_ID reads 0x5449.
3. Read BUS_VOLTAGE and CURRENT registers, convert (×1.25). Simultaneously read the DMM:
   voltage at the load, then DMM in series (10 A range) for current.
   Expected: INA260-vs-DMM agreement **within ±5 %** on both (G1.4). Record both numbers.
4. Reverse-direction sanity: swap VIN+/VIN− briefly with the resistor load — CURRENT must
   read negative, same magnitude. (Catches a backwards install before it lies to you for a
   month.) Restore correct orientation.
5. Threshold demo (G1.5): set ALERT_LIMIT for bus-UV at a value just above the bench rail
   (e.g., 11.5 V on a 12.0 V rail → no alert; 12.5 V → alert asserts). Observe MASK/ENABLE
   flag (and pin, if wired) flip across the boundary. If using strategy A only: demonstrate
   the same with firmware-printed threshold state while trimming the buck ± 0.5 V.
6. Install at final locations (M0 at buck input, M1/M2 per decision), re-run step 3 in place
   with one servo sweeping. Expected: vbus at M0 ≈ pack voltage (sags with load, per
   TBD-10); current consistent with P1-01 step-8 baseline.

## Acceptance gates
- **G1.4** — every installed INA260 agrees with the DMM within **±5 %** on voltage AND
  current at a known load; reversed-polarity check passed; addresses unique and labeled.
- **G1.5** — a threshold crossing (alert pin OR polled flag, per chosen strategy) is
  demonstrably observed at the configured value ± 1 LSB-rounding, latched/cleared correctly.

## Fallback ladder
- **A:** INA260 per the placement table (M0 mandatory).
  *Switch per-path to B if:* that path's worst-case exceeds 15 A (measured, not feared), or
  an INA260 unit fails G1.4 twice with wiring verified.
- **B:** INA226 + external shunt on the offending path (sizing above); INA260s remain
  everywhere they fit. *Switch to C if:* I2C monitoring itself proves unreliable (bus
  lockups under motor noise after pull-up/cable fixes, 2 sessions).
- **C:** minimum-viable brownout protection: Teensy ADC resistor divider on the pack
  (e.g., 100 kΩ : 18 kΩ → 17.4 V → 2.65 V < 3.3 V ADC ceiling; verify values on bench, 1 %
  resistors) for vbus only; `current` field reports 0 with a research-log note; current
  monitoring deferred to a bench clamp meter during tests. The brownout ladder MUST work
  even on plan C — sit-down beats telemetry richness.

## Rollback
Remove monitor(s) from the current path, bridge with an equivalent-gauge jumper, re-run
P1-01 G1.1/G1.2 to confirm the tree is unchanged. Monitoring is removable by design; never
let a dead monitor strand the robot unpowered.

## Artifacts → docs/05_RESEARCH_LOG.md
Chosen placement set + addresses; G1.4 table (INA260 vs DMM, per unit); G1.5 evidence;
CONFIG register value chosen; alert strategy decision; any INA226 fallback design with shunt
value, CAL math, and its own G1.4-equivalent result.

## If this entire phase approach fails
If digital power monitoring cannot be made trustworthy at all (chips dead, I2C hopeless):
run plan C's divider for vbus (brownout safety is non-negotiable), strap a panel ammeter or
clamp meter onto the bench for every powered session, set fault bit2 from the divider alone,
and ship the robot with conservative thresholds raised by the unmeasured-sag margin
(warn 14.2 / alert 14.0 / floor 13.8 V) until real current telemetry exists. Log the
degradation prominently in docs/01_STATUS.md — P4 gait-power tuning is blocked without
current data, walking is not.
