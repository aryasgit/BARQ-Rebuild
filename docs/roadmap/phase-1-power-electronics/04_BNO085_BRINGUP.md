# P1-04 — BNO085 IMU Bring-up: interface ladder, mounting, bench validation

> Phase P1 · verified against repo @ 0e5ddaf

## Objective
Bring the owned BNO085 up on the bench attached to the Teensy 4.1, choose its interface by
ladder (I2C SH-2 → SPI → UART-RVC), verify axes against the REP-103 body frame the whole
stack assumes, and prove a stable 100 Hz stream. The protocol STATE frame wants
`quat[4] + gyro[3] + accel[3]` at 100 Hz (docs/06_PROTOCOL.md); P3 fills the `imu_read()`
stub (`barq_firmware/src/loop_core.cpp` — currently returns identity + 981 cm/s² on Z, which
is exactly what a correctly mounted, level robot must reproduce for real).

## Prerequisites
- P1-03 executed (pin budget below assumes its serial selection; Teensy alive on USB).
- BNO085 breakout with PS0/PS1 straps and INT/RST broken out (owned). Spare-unit criteria if
  one must be bought: same chip, straps + INT/RST accessible — ₹1,200–3,500.
- Host with PlatformIO (P0); an SH-2 capable Arduino library for the bench sketch (the P3
  doc owns the production driver; any maintained BNO08x SH-2 library proves hardware).
- 3.3 V logic only — the BNO085 and Teensy 4.1 are both 3.3 V parts; never 5 V.

## Interface decision ladder (with mode straps)
Mode straps (public BNO08x table — VERIFY against the CEVA/Hillcrest datasheet and your
breakout's silkscreen; some boards label them P0/P1 or use solder jumpers):
| PS1 | PS0 | Mode |
|---|---|---|
| 0 | 0 | **I2C** (ladder A) |
| 0 | 1 | **UART-RVC** (ladder C) |
| 1 | 0 | UART-SHTP (not used — SH-2 over UART, library support weakest) |
| 1 | 1 | **SPI** (ladder B) |

- **A — I2C + SH-2 (default):** full rotation vector + calibrated gyro + accel — everything
  the STATE frame wants. Address **0x4A** (default) or **0x4B** (address pin/jumper high) —
  public values, verify against the breakout schematic. INT and RST wired (recommended for
  I2C, mandatory discipline anyway — SH-2 is interrupt-paced and RST recovers lockups
  without a power cycle).
- **B — SPI:** same full SH-2 data; switch if I2C proves flaky under motor noise. INT is
  MANDATORY in SPI (transactions are INT-gated by the SH-2 protocol), RST mandatory.
- **C — UART-RVC:** dead-simple fixed 100 Hz ASCII-of-binary stream, 115200 8N1 — but
  REDUCED data (see "what C loses" below). Adopt only if SH-2 (A and B) cannot be made
  reliable, or as the P3 quick-start while SH-2 driver work proceeds.

*Switch criteria A→B:* I2C lockups/NACK storms or stalled streams under servo load that
survive: shorter leads, twisted pair with ground, 4.7 kΩ pull-up check, 100 kHz fallback
clock, and separate-bus isolation — across 2 sessions. *Switch B→C:* SPI still unstable
(after INT-gating verified correct) or SH-2 library defects unresolved after 2 sessions —
also see P3's escape hatch; C is the guaranteed-motion fallback.

## Wiring per option (Teensy 4.1 pin budget consistent with P1-02/P1-03)
```
A: I2C (PS1=0, PS0=0)                B: SPI (PS1=1, PS0=1)
Teensy 3V3  --- VIN/3V3              Teensy 3V3  --- VIN/3V3
Teensy GND  --- GND                  Teensy GND  --- GND
pin 17 SDA1 --- SDA  [4.7k to 3V3]   pin 13 SCK  --- SCK
pin 16 SCL1 --- SCL  [4.7k to 3V3]   pin 12 MISO --- (board DO/MISO/SA0 pad)
pin 2       --- INT (active low)     pin 11 MOSI --- (board DI/MOSI/SDA pad)
pin 3       --- RST                  pin 10 CS   --- CS
(addr pin low = 0x4A, high = 0x4B)   pin 2  INT, pin 3 RST  (both REQUIRED)
                                     (SPI pad naming varies by breakout —
Bus is Wire1 — reserved free by       match the board silk to the datasheet
P1-03's serial selection; INA260s     before wiring, then verify by probe)
stay on Wire (18/19): an IMU
lockup cannot stall power telemetry.

C: UART-RVC (PS1=0, PS0=1)
Teensy 3V3 --- VIN/3V3 ; GND --- GND
pin 15 RX3 <-- board TX   (Serial3 = the spare kept free in P1-03)
115200 8N1, board streams 19-byte frames at a fixed 100 Hz, no commands needed
(RST to pin 3 still recommended)
```
All IMU leads: 24–28 AWG, twisted with ground, ≤ 30 cm, routed AWAY from servo power
harness (cross at 90° if they must meet).

## What C (UART-RVC) loses — written down so nobody is surprised in P4
RVC frame carries: index, **yaw/pitch/roll (int16, 0.01°)**, **linear accel x/y/z**, checksum
(19-byte frame, header 0xAAAA — verify field order/scale against the datasheet appendix).
It does NOT carry: quaternion (must be converted from Euler — fine near level, ambiguous at
pitch ±90°), **gyro rates (gone entirely)**, calibration status, magnetometer.
Consequences for the stack: `quat[4]` = Euler→quat conversion (acceptable for walking
poses); `gyro[3]` must be synthesized by differentiating Euler angles — noisy, wrap-around
hazards at ±180°, and the P4 estimator loses its clean angular-velocity input (degraded
push-recovery/turn-rate feedback). `accel[3]` fine. **fault bit1 (IMU stale) semantics
survive unchanged:** stale = no valid frame for N ms, regardless of interface — the 100 Hz
fixed cadence actually makes staleness detection trivial (any gap > ~25 ms is abnormal).
Log adoption of C as a decision with this paragraph referenced.

## Mounting orientation (REP-103 vs sensor axes)
- Body frame (REP-103, used by sim/URDF/controllers): **X forward, Y left, Z up**.
- The URDF mounts the sim IMU directly on `base_link` (barq.urdf.xacro, `<sensor name="imu">`,
  `ignition_frame_id base_link`, 100 Hz) — i.e., the stack assumes IMU frame ≡ body frame.
  Physically mount the breakout so its silkscreen axes align with body axes (sensor X →
  nose). If perfect alignment is impossible, ANY 90°-multiple mounting is fine — but the
  remap then lives in exactly ONE place (P3 firmware, before packing STATE) and is recorded.
- Mount: rigid, near the body center, away from the servo power harness; non-ferrous
  fasteners near the magnetometer; if the tap test (below) shows ringing, add a thin
  foam-tape layer and re-test.
- **TBD-15 — final mount transform** (translation [mm] + axis remap matrix, measured at
  install during P2 assembly): goes into the research log AND the P3 imu_read remap (and
  the URDF if translation matters for estimation later).

## Bench validation procedure (with expected readings)
Run a minimal SH-2 example streaming rotation vector + gyro + accel at 100 Hz as CSV over
USB serial (or the RVC stream if on C). Board flat on the bench, "nose" (sensor +X) marked
with tape pointing away from you = "forward".
1. **Flat-table sanity (2 min settle):** quat ≈ identity up to yaw: roll/pitch ≤ 2°
   equivalent; w-dominant quat (|w| > 0.99 after zeroing yaw). Yaw is magnetometer-referenced
   and will settle/creep — judge roll/pitch only. Accel ≈ (0, 0, **+981**) cm/s² ± 30;
   gyro magnitude < 30 mrad/s (typically < 10).
2. **90° rotation signs table** (do each from the same level start pose; compare the CHANGE;
   quat sign pairs (x,y,z,w) ≡ (−x,−y,−z,−w) are the same rotation — normalize to w ≥ 0):
| Maneuver (right-hand rule, REP-103) | Expected quat (x,y,z,w) | Gyro during motion | Accel at rest after |
|---|---|---|---|
| level start | (0, 0, 0, 1) | ≈ 0 | (0, 0, +981) |
| yaw **+90°** = nose swings LEFT (CCW from above) | (0, 0, +0.707, 0.707) | **gz > 0** | (0, 0, +981) |
| roll **+90°** = RIGHT side down (left side rises) | (+0.707, 0, 0, 0.707) | **gx > 0** | (0, +981, 0) |
| pitch **+90°** = NOSE DOWN (REP-103 quirk: +pitch about +Y-left is nose-down) | (0, +0.707, 0, 0.707) | **gy > 0** | (−981, 0, 0) |
   Tolerance: quat components ± 0.05; gyro sign unambiguous during the move; accel within
   ± 50 cm/s² of the listed axis. ANY sign mismatch = mounting/remap error — fix the remap
   (one place!), never "fix" it downstream in the estimator.
3. **Vibration sanity (tap test):** stream at 100 Hz, tap the bench 5× near the board:
   accel shows transients; quat deviates < 1° and recovers < 1 s; NO reset events. Then hold
   the board against a sweeping servo's bracket (`st3215_diag.py sweep`) for 60 s: stream
   unbroken, no resets, roll/pitch noise < 1° RMS-equivalent.
4. **Endurance (the gate):** 10 min continuous at 100 Hz. Count: delivered frames (expect
   ≈ 60,000 ± 1 %), reset indications (SH-2 reports a reset/"product ID response" on
   restart; RVC index discontinuities), I2C/SPI errors. Expect ZERO resets.

## TBD table
| ID | Value | Procedure |
|---|---|---|
| TBD-15 | final mount transform (translation + axis remap) | measure at install (P2), log + apply in P3 remap |
| TBD-16 | chosen interface rung (A/B/C) + bus speed | this ladder's outcome |
| TBD-17 | observed reset count / 10 min under servo noise | step 3–4 with servos running (repeat after install) |

## Acceptance gate
- **G1.9** — stable **100 Hz for 10 min** (frame count within 1 %), **zero resets**,
  flat-table sanity passed, and the **90° signs table verified for all three axes** (the
  table above, all rows). On ladder C, the same gate applies with the Euler-derived quat
  and the gyro rows replaced by "yaw/pitch/roll deltas match maneuver signs".

## Fallback ladder (hardware)
- **Board dead** (no I2C ACK at 0x4A/0x4B, no RVC stream, after: 3V3 present, straps
  re-checked, RST pulsed, second cable): swap to a second unit (buy to the criteria above if
  no spare on hand) — switch after 30 min of the listed checks, not before.
- **Persistent I2C lockups** under motor noise → ladder B (SPI), straps 1/1, INT-gated.
- **SH-2 library issues at P3** (driver integration, not hardware) → ladder C (UART-RVC)
  with the degradation paragraph above attached to the decision; revisit SH-2 when P3
  stabilizes.

## Rollback
Straps back to A (0/0), board off the robot, back on the bench loom — the bench CSV sketch
is the permanent lowest-layer IMU test point. Re-run step 1 after ANY strap/wiring change.

## Artifacts → docs/05_RESEARCH_LOG.md
TBD-15/16/17; the signs-table results (measured numbers, all rows); endurance counts;
reset events with timestamps; interface decision + switch evidence if A was abandoned;
photos of the mount + strap settings.

## If this entire phase approach fails
If no BNO085 interface yields a stable stream on two units: substitute a commodity IMU
class (ICM-20948 / BMI270 / MPU-6050-era breakout, ₹200–1,500) and run Madgwick/Mahony
fusion ON the Teensy. The protocol frame is unchanged (quat/gyro/accel fields are
chip-agnostic); costs: no on-chip SH-2 fusion quality, magnetometer-free yaw drifts, P4
estimator must trust feet+gyro more — record as a major decision in docs/02_DECISIONS.md
and re-run THIS file's signs table and endurance gate on the substitute before P3 consumes
it.
