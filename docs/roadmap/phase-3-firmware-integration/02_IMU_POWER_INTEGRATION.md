# P3-02 — IMU (BNO085) + Power (INA260) Integration (fill `imu_read` / `power_read`)

> Phase P3 · verified against repo @ 0e5ddaf

## Objective
Fill the remaining LoopCore stubs behind the P3-01 `HwIface` seam: BNO085 orientation/gyro/
accel into the STATE frame in protocol units (quat ×1e-4 **x,y,z,w — w LAST**, gyro mrad/s,
accel cm/s²) with a 50 ms staleness watchdog → fault bit1; INA260 bus voltage/current at
10 Hz → vbus mV / current mA, alert threshold → fault bit2, and the 13.2 V hard floor where
the firmware itself goes limp. LoopCore stays pure C++ (all chip code in `teensy_hw.cpp` /
new `imu_bno085.cpp`, `power_ina260.cpp`, Teensy env only).

Naming note (one-time): older docs (06_PROTOCOL.md "(INA226)", barq_firmware README/main.cpp
comments) say INA226 — superseded by the roadmap README hard truth: **INA260s are owned**
(integrated 2 mΩ shunt, ±15 A/unit). The protocol fields are chip-agnostic; do not change the
frame. INA226 + external shunt remains the documented fallback (P1-02).

## Prerequisites
- P3-01 merged and G3.1 passed (the HwIface seam exists; debug CDC `SerialUSB1` works).
- P1-04 done: BNO085 mounted, I2C wired to the Teensy (which Wire port + pullups recorded
  there), mount transform body→IMU **measured** (P1-04's output is the quaternion below).
- P1-02 done: INA260 placement + I2C address strap table (base 0x40 + A0/A1), which monitor
  is the battery-side canonical one.
- `gen_firmware_config.py` from P3-01 (this file adds fields to its output).

## 1. `imu_read` — BNO085 over SH-2 / I2C (primary mode)
**Library options** (search the PlatformIO registry, verify maintained + license before
pinning in `platformio.ini` `lib_deps`):
- `adafruit/Adafruit BNO08x` — wraps CEVA's official `sh2` C driver; Adafruit wrapper
  BSD/MIT-style, embedded sh2 driver has its own (Apache-like) license — read both LICENSE
  files, record in the research log.
- `sparkfun/SparkFun BNO080 Cortex Based IMU` — same silicon family, MIT.
Either works; prefer whichever builds clean for teensy41 first. Use library enums
(`SH2_ROTATION_VECTOR` etc.) — do NOT hand-roll SH-2 report IDs (none are proven in-repo).

**Init sequence (behavioral spec):**
1. `Wire` port + 400 kHz per P1-04; BNO085 default I2C address 0x4A (0x4B alt) — public value,
   verify against the wiring (TBD-6). BNO085 clock-stretches; if the Teensy i.MXRT I2C
   misbehaves, first drop to 100 kHz (ladder).
2. begin/probe; on failure and `kImuFitted`: log on debug CDC, keep stub values, **set bit1**
   (IMU promised but absent = stale).
3. Enable three reports, 10 000 µs interval each (= 100 Hz): **rotation vector** (the
   06_PROTOCOL.md contract), **calibrated gyro**, **accelerometer** (gravity-INCLUDED — the v0
   stub's flat-table value is accel z = +981 cm/s², keep that convention; not linear-accel).
4. Pump the SH-2 service from `TeensyHw::poll()` every superloop tick (non-blocking; typically
   `getSensorEvent()` until empty). Cache the latest of each report + its `micros()` stamp.
5. **Reset handling**: the BNO085 silently resets under brownout/ESD; the libs expose
   `wasReset()`. On reset: re-enable all three reports, increment a counter (debug CDC), keep
   bit1 until the first fresh rotation vector arrives.

**Mapping/scaling to protocol units** (`imu_read` copies from cache; clamp everything to
int16 with a saturating helper):

| STATE field | Source (SH-2) | Conversion | Trap |
|---|---|---|---|
| quat[4] ×1e-4 | rotation vector i,j,k,real | x=i, y=j, z=k, **w=real**, each ×10 000 | protocol order is **x,y,z,w (w-last)**; SH-2/Adafruit name the scalar `real` — the v0 stub identity `{0,0,0,10000}` pins w-last; a w-first mistake passes flat-table and explodes in P4's estimator |
| gyro[3] mrad/s | calibrated gyro, rad/s float | ×1000, clamp ±32767 (saturates ≥ 1877 °/s — irrelevant for walking, clamp anyway) | sign check in G3.3 |
| accel[3] cm/s² | accelerometer, m/s² float | ×100 | flat table must read ≈ (0,0,+981) |

**Mounting-frame rotation hook** (P1-04's measured mount transform, compile-time):
`firmware_config.h` gains `kImuMountQuat1e4` = q_ib (rotation of the BODY frame expressed in
the IMU frame; identity if the chip's axes already align with body x-fwd/y-left/z-up).
Compose **before** scaling:
```
q_wb = q_wi ⊗ q_ib            (Hamilton product, components stored x,y,z,w):
w = w1w2 − x1x2 − y1y2 − z1z2
x = w1x2 + x1w2 + y1z2 − z1y2
y = w1y2 − x1z2 + y1w2 + z1x2
z = w1z2 + x1y2 − y1x2 + z1w2          (1 = measured q_wi, 2 = mount q_ib)
```
Gyro/accel are VECTORS in the IMU frame → rotate into body with the inverse:
`v_body = R(q_ib)ᵀ · v_imu`. For the overwhelmingly common axis-aligned mount (multiples of
90°) the generator also emits `kImuMountMat3` (int8 entries ∈ {−1,0,1}) computed from the
quat, and the firmware does 9 multiplies — no float quat-rotate in the hot path. The generator
errors out if the supplied quat is not axis-aligned within 2° (then either fix the mount or
implement the float path — note it as a deviation).

**Staleness watchdog** (lives in LoopCore so it's host-testable): `imu_read` (HwIface) returns
`true` only when a rotation-vector report newer than the last consumed one exists. LoopCore
tracks `last_imu_ok_us`; **no fresh report for 50 ms → fault bit1 + keep serving last-good
values** (never zeros — a zero quat is poison downstream). bit1 clears on the next fresh
report.

**UART-RVC fallback mode** (ladder rung — wiring change: PS0/PS1 straps + one spare Teensy
UART RX @115200, P1-04 documents the strap): the BNO085 then streams fixed 19-byte frames at
100 Hz containing **yaw/pitch/roll (centi-deg) + accel** only. What changes in firmware: a
~40-line frame parser replaces the SH-2 lib; quat reconstructed from euler (give the standard
ZYX→quat formula in code comments); **what the estimator loses: the gyro entirely** (gyro[3]
sent as 0 + a debug-CDC banner; do NOT fake it by differentiating euler), magnetometer-free
yaw drift characteristics change, and no dynamic-calibration status. D-017's estimator uses
quat-yaw + joint FK, so it degrades but functions. Record the mode in STATE? No frame change —
research-log + banner only.

## 2. `power_read` — INA260 over I2C
Register table — **public TI datasheet values, NOT proven in-repo: verify against the INA260
datasheet (§8.6) before first use** (the diag-script rule: nothing here was bench-proven yet):

| Reg | Addr | Note |
|---|---|---|
| CONFIG | 0x00 | default continuous bus+current is fine; averaging 4 recommended |
| CURRENT | 0x01 | **LSB 1.25 mA**, two's complement |
| BUS VOLTAGE | 0x02 | **LSB 1.25 mV** |
| POWER | 0x03 | LSB 10 mW (unused v1) |
| MASK/ENABLE | 0x06 | only if using the ALERT pin (not required v1 — we poll) |
| ALERT LIMIT | 0x07 | idem |
| MFR/DIE ID | 0xFE/0xFF | probe check: MFR = 0x5449 ("TI") |

**Trap:** INA260 registers are **big-endian (MSB first)** on the wire — opposite of the
Feetech bus. Address per monitor from **P1-02** (0x40 + A0/A1 straps); the **battery-side
monitor is the canonical source** for STATE vbus/current. If P1-02 fitted extra monitors
(per-rail), v1 firmware may poll them for the alert only — STATE has one pair; per-rail
telemetry is a future DIAG frame, same policy as the servo stale bitmap (P3-01 §8).

**Polling**: 10 Hz from `TeensyHw::poll()` (two 16-bit reads ≈ trivial at 400 kHz; never in
the 500 Hz hot path). Convert: `vbus_mv = raw × 1.25` (uint16 holds up to 65.5 V, 4S max
17.1 V fine), `current_ma = (int16)raw × 1.25` (±15 A ceiling per unit, int16 mA caps ±32.7 A
fine). Cache + timestamp; `power_read` copies the cache.

**Thresholds and the reaction split — state this design explicitly:**
- `vbus < kVbusAlertMv` (**default 13 800 mV**, from P1; sits deliberately above the 13.6 V
  operational floor in the roadmap README) for **3 consecutive samples (300 ms debounce** —
  trot current spikes sag the pack; a single dip must not trip) → set **fault bit2**.
  **Firmware only FLAGS.** The Jetson-side safety layer (P4 safety doc) owns the reaction —
  controlled sit-down request, mission abort, etc. Rationale: the firmware cannot sequence a
  graceful sit (that needs IK/gait state); it must not fight the planner.
- **EXCEPT the hard floor: `vbus < kVbusFloorMv` (13 200 mV) for 5 consecutive samples
  (500 ms)** → the firmware ITSELF forces torque-off via the P3-01 edge path
  (`power_floor_tripped()` → LoopCore forces not-driven → limp; robot sits/collapses softly —
  designed behavior). bit2 stays set; recovery requires vbus back above floor + fresh CMD
  stream. Below 3.3 V/cell, waiting for the Jetson is how packs die — the local reflex is
  non-negotiable, mirroring the deadman philosophy.
- **Sensor-absent policy**: if `kPowerFitted` and the probe fails (or reads go stale > 1 s):
  vbus/current report 0, **bit2 SET** (power not verified), but the **hard floor is DISABLED**
  (a dead sensor reading 0 mV must not torque-off a healthy robot); deadman still protects.
  If `!kPowerFitted` (bench builds): stub values (v0's 7400 mV), bit2 clear, banner on
  debug CDC.
- **Bench trap (will bite in P3-03)**: a 12 V bench PSU on the monitored rail is below BOTH
  thresholds → instant bit2 + floor torque-off. Use `gen_firmware_config.py --bench-power`
  (alert 11 500 / floor 10 500 mV) for all bench work; the flag stamps `BENCH-POWER` into the
  config banner so a robot build can't ship with it silently (G3.4 checks the banner).

`firmware_config.h` additions (generator flags): `kImuFitted`, `kPowerFitted`,
`kImuMountQuat1e4[4]`, `kImuMountMat3[9]`, `kIna260Addr`, `kVbusAlertMv`, `kVbusFloorMv`.

## Acceptance gates
**G3.3 — IMU sanity + 90° sign table through the FULL chain.** Not bench prints: firmware
flashed, BNO085 on the robot/bench plate, **observed in the STATE frame on the Jetson** via
`diagnostics/state_peek.py --port /dev/ttyACM0` (P3-01 tool; passive mode — bit3 set is
expected and irrelevant here). Body frame: x fwd, y left, z up (REP-103). Extract
roll = atan2(2(qw·qx+qy·qz), 1−2(qx²+qy²)), pitch = asin(2(qw·qy−qx·qz)) — x,y,z,w order.

| # | Maneuver | PASS bars (all of) |
|---|---|---|
| 1 | flat table, still, 60 s | accel = (0,0,+981) ±30 cm/s²; gyro all |·| < 20 mrad/s; |roll|,|pitch| < 3°; norm(quat) = 1.0 ±0.01; bit1 = 0; no value frozen (LSB jitter visible) |
| 2 | nose straight up (x up), held | accel ≈ (+981,0,0) ±60 |
| 3 | left side up (y up), held | accel ≈ (0,+981,0) ±60 |
| 4 | upside down | accel ≈ (0,0,−981) ±60 |
| 5 | rotate: nose tips DOWN (about +y) | gyro y > 0 during motion (ROS y-left makes nose-DOWN the positive pitch rate — classic sign trap, do not "fix" it to aerospace) |
| 6 | roll RIGHT side down (about +x) | gyro x > 0 during motion |
| 7 | yaw nose LEFT / CCW from above | gyro z > 0 during motion; quat-yaw increases |
| 8 | unplug SDA mid-run (or hold reset) | bit1 sets within ~60 ms, values hold last-good (no zeros); reconnect → bit1 clears, values resume |

Rows 2–7 are evaluated AFTER the mount rotation hook — they validate kImuMountQuat, not the
bare chip. Any sign flip → fix the mount quat in P1-04's record + regenerate, never patch
signs in code.

**G3.4 — INA values ±5 % of a multimeter under known load.** Monitored rail from bench PSU
(`--bench-power` build), known load drawing 2–5 A (3 servos mid-sweep, or a power resistor).
Simultaneously: DMM across the rail (voltage) and DMM 10 A range in series (current).
PASS: `state_peek` vbus within ±5 % of DMM volts AND current within ±5 % of DMM amps at two
load points (idle ≤ 0.5 A and the 2–5 A point); config banner shows the expected
bench/robot power profile. Also force a sag below the (bench) alert: bit2 sets after ~300 ms,
clears on recovery; drop below (bench) floor 500 ms → servos go limp (watch a held joint).

## Fallback ladder (switch after 2 failed attempts or 2 h per rung)
**IMU:** A) SH-2/I2C 400 kHz → B) I2C 100 kHz (clock-stretch tolerance) → C) **UART-RVC mode**
(§1; gyro lost — recorded as a standing TBD against P4 estimator quality). Magnetic yaw jumps
near motors (seen as quat-yaw steps at servo enable): switch report to GAME rotation vector
(no magnetometer) — same code path, one enum; note in research log + a D-number since it
changes yaw semantics to relative.
**Power:** A) INA260 per P1-02 → B) other I2C address/port (strap audit, probe scan 0x40–0x4F
printed on debug CDC) → C) INA226 + external shunt (the P1-02 documented fallback; different
LSBs — re-derive conversion, registers TBD against ITS datasheet) → D) no monitor: ship bench
sessions only, `kPowerFitted=false`, **never field-walk without a working floor cutoff**
(hard rule; the 13.6 V operational floor would otherwise be unenforced).

## Rollback
Regenerate config with `kImuFitted=false` / `kPowerFitted=false` and re-flash (stubs return,
bits 1/2 clear) — subsystem rollback without touching P3-01's servo driver. Full rollback:
archived v0 hex as in P3-01.

## TBD table
| # | Value | Producing procedure |
|---|---|---|
| TBD-6 | BNO085 I2C address as wired (0x4A/0x4B) | P1-04 strap record; probe scan on debug CDC |
| TBD-7 | chosen lib id + version + licenses | PlatformIO registry search; pin exact version in platformio.ini |
| TBD-8 | INA260 register table vs datasheet | read MFR_ID (expect 0x5449) + one known-voltage check before trusting LSBs |
| TBD-9 | alert/floor debounce under real trot sag | P3-03 air-walk: log vbus dips; lengthen debounce only with data |
| TBD-10 | mount quat (P1-04 measured) | P1-04 procedure; G3.3 rows 2–7 are its verification |

## Artifacts → docs/05_RESEARCH_LOG.md
G3.3 table filled with measured numbers (photo of each pose + the state_peek line),
`state_peek --csv` traces into `artifacts/`, G3.4 DMM-vs-INA table at both load points,
lib versions/licenses, reset-counter observations, updated `firmware_config.h` committed.
Log entry per the standing practice (believed/measured/changed); the INA226→INA260 naming
correction is worth one line against docs/06_PROTOCOL.md history.

## Escape hatch
If the BNO085 cannot be made reliable on the Teensy at all (I2C and RVC both flaky): move the
IMU to the **Jetson's own I2C header** and publish `/imu/data` from a Python node — STATE's
IMU fields then stay stubbed (bit1 policy: clear + banner, since the IMU is no longer
promised on this link) and the estimator consumes the Jetson topic directly (it already
subscribes to `/imu/data` per D-017). Costs: IMU leaves the 500 Hz loop, no firmware-side
orientation for future reflexes; acceptable for P4, revisit before RL. Power equivalent: a
Jetson-side USB/I2C meter for logging, but the 13.2 V reflex is lost — then the operational
floor must be enforced procedurally (timer + pre-flight voltage check, P7 runbook) and the
pack protected by its own low-voltage alarm. Write either move up as a D-number first.
