# P3-01 — ST3215 Servo Bus Driver (fill `servo_bus_*` stubs)

> Phase P3 · verified against repo @ 4ea53a0

## Objective
Replace the loopback `servo_bus_write_targets` / `servo_bus_read_state` stubs in
`barq_firmware/src/loop_core.{h,cpp}` with a real Feetech STS bus driver: 12 ST3215s on
4 UARTs (3 per leg-bus) at 1 Mbaud, sync-written targets every 500 Hz tick, positions/vel/load
read round-robin so every 100 Hz STATE frame carries servo data ≤ 6 ms old — **without breaking
LoopCore's pure-C++ host build** (the emulator and `integration_pty.py` 9/9 must keep passing
untouched).

## Prerequisites
- P1-03 done: 4 bus harnesses built, duplex mode determined (half single-wire vs full with
  combiner), Teensy `Serial<n>` ↔ leg mapping recorded. **Reference P1-03's table — do not
  re-derive it here; the firmware copies it into one array (§5).**
- P2 done for at least 3 servos of one leg: IDs assigned per the plan in
  `diagnostics/st3215_diag.py` (`ID_PLAN`: FL 0-2, FR 3-5, RL 6-8, RR 9-11), mid-calibrated
  (torque-enable=128 trick), `artifacts/servo_calib.yaml` written.
- Bench: 12 V servo rail from P1-01 buck (NOT USB), master switch in reach, servos clamped or
  horn-free. `pio test -e native` 6/6 and `integration_pty.py` 9/9 green before you start.

## 1. Architecture rule (read twice)
`loop_core.{h,cpp}` is compiled **verbatim** by two builds: PlatformIO `teensy41` and the host
CMake build of `barq_hw/teensy_emulator`. Therefore:

- **No Arduino types/headers ever enter `loop_core.h/.cpp`.** All hardware goes behind an
  injected interface (§2). `HardwareSerial`, `Wire`, `micros()` live only in `main.cpp` and new
  Teensy-only files (`teensy_hw.{h,cpp}`, `bus_driver.{h,cpp}` under `barq_firmware/src/`,
  excluded from the native test env — `platformio.ini` native already filters
  `build_src_filter = +<protocol.cpp>`; the emulator's CMake lists `loop_core.cpp` only).
- With no hardware injected, LoopCore behaves **byte-identically to v0 loopback** — that is the
  regression anchor for G3.5.

## 2. The seam: `HwIface` injected into LoopCore
Add to `loop_core.h` (pure C++, stdint only):

```cpp
struct HwIface {                       // implemented by TeensyHw (Arduino side only)
  virtual ~HwIface() = default;
  virtual void servo_write_targets(const int16_t mrad[12]) = 0;   // stage sync-writes
  virtual void servo_torque(bool on) = 0;                          // all fitted servos
  virtual void servo_read(int16_t pos[12], int16_t vel[12], int16_t load[12],
                          int8_t* temp_max, bool* bus_fault) = 0;  // copy cached, no bus I/O
  virtual bool imu_read(int16_t quat[4], int16_t gyro[3], int16_t accel[3]) = 0; // P3-02
  virtual bool power_read(uint16_t* vbus_mv, int16_t* current_ma, bool* alert) = 0; // P3-02
  virtual bool power_floor_tripped() = 0;                          // 13.2 V hard floor, P3-02
};
class LoopCore {
 public:
  explicit LoopCore(HwIface* hw = nullptr);   // nullptr = v0 loopback (emulator, tests)
  ...
};
```

LoopCore changes (the entire diff is ~25 lines — keep it that small):
- `tick()`: track `was_fresh_`; on rising edge (`!was_fresh_ && fresh`) call
  `hw_->servo_torque(true)`; on falling edge (deadman trip) call `hw_->servo_torque(false)`.
  If `hw_ && hw_->power_floor_tripped()`, force `fresh = false` (drives the same torque-off
  edge — one code path for "go limp"). With `hw_ == nullptr` nothing new executes.
- `send_state()`: when `hw_` is set, source pos/vel/load/temp/fault-bit0 from
  `hw_->servo_read(...)`, IMU/power from the P3-02 calls; fault byte =
  `bit0(bus_fault) | bit1(imu stale, §P3-02) | bit2(power alert) | bit3(deadman)`.
- Safety semantics (explicit, by design): deadman / torque-off writes
  `REG_TORQUE_ENABLE = 0` to every fitted servo → **servos go limp; the robot sits/collapses
  softly on the cradle or floor.** That is preferred over a rigid hold on faults. Never gate it.

`main.cpp` becomes:
```cpp
TeensyHw hw;                 // owns BusDriver[4], later Bno085+Ina260 (P3-02)
barq::LoopCore core(&hw);
void loop() {
  const uint32_t now = micros();
  hw.poll(now);              // bus state machines advance; parse replies; issue next reads
  while (Serial.available() > 0) core.rx_byte(Serial.read(), now, tx, nullptr);
  core.tick(now, tx, nullptr);
  /* deadline pacing: spin-wait to the 2000 µs tick edge; never block inside hw.poll() */
}
```
Recommended (not gating): a new native test `test/test_loop_core/` with a scripted FakeHw
asserting torque(true) fires exactly once per fresh edge and torque(false) on deadman/floor.

## 3. Feetech STS protocol — what the repo has already proven
Source of truth: **`diagnostics/st3215_diag.py`** (bench-proven against real ST3215s through a
Waveshare USB adapter). Reuse its constants verbatim; anything NOT in that file is marked TBD
and must be verified against the Feetech STS/ST3215 datasheet (or the script, extended on the
bench) before use.

Instruction frame (master → servo), from `Bus._txrx`:
```
0xFF 0xFF | id | len | instr | params... | chk        len = nparams + 2
chk = (~(id + len + instr + Σparams)) & 0xFF
```
Status frame (servo → master): `0xFF 0xFF | id | len | err | data... | chk`, `len = ndata + 2`,
same checksum over `id, len, err, data`. **Multi-byte registers are little-endian** (low byte
first — `read16`/`write16` in the script). Speed/load/current use 15-bit sign: bit15 set ⇒
negative of (v & 0x7FFF) (`s15()`).

Instructions proven in-repo: `PING 0x01`, `READ 0x02`, `WRITE 0x03`.
**`SYNC_WRITE 0x83` is NOT in the script** → TBD-1: verify opcode + layout on the bench (§6)
before relying on it; fallback is per-servo WRITE (ladder §Fallbacks).

Register table (every row below is lifted from `st3215_diag.py` — cite it in code comments):

| Name | Addr | Size | Notes (from the script) |
|---|---|---|---|
| EPROM_ID | 0x05 | 1 | bus ID; unlock 0x37 first |
| EPROM_BAUD | 0x06 | 1 | value↔baud table NOT in script → TBD-2 |
| EPROM_MIN_ANGLE | 0x09 | 2 | counts |
| EPROM_MAX_ANGLE | 0x0B | 2 | counts |
| EPROM_MAX_TEMP | 0x0D | 1 | °C |
| EPROM_MAX_VOLT / MIN_VOLT | 0x0E / 0x0F | 1 | 0.1 V units |
| EPROM_MAX_TORQUE | 0x10 | 2 | 0.1 % units |
| EPROM_OFS | 0x1F | 2 | position offset (mid-cal writes this internally) |
| EPROM_MODE | 0x21 | 1 | 0=position (required) /1 wheel /2 PWM /3 step |
| REG_TORQUE_ENABLE | 0x28 | 1 | 0 off / 1 on / **128 = calibrate current pose as 2048** |
| REG_ACC | 0x29 | 1 | acceleration |
| REG_GOAL_POS | 0x2A | 2 | counts |
| REG_GOAL_SPEED | 0x2E | 2 | 0 = max (the proven 100 Hz streaming mode, `cmd_sweep`) |
| REG_LOCK | 0x37 | 1 | EPROM lock: 0 unlock / 1 lock |
| REG_PRESENT_POS | 0x38 | 2 | counts |
| REG_PRESENT_SPEED | 0x3A | 2 | s15; units ≈ counts/s → TBD-3 verify scale |
| REG_PRESENT_LOAD | 0x3C | 2 | s15, ~0.1 % units (matches STATE load field directly) |
| REG_PRESENT_VOLT | 0x3E | 1 | 0.1 V |
| REG_PRESENT_TEMP | 0x3F | 1 | °C |
| REG_MOVING | 0x42 | 1 | 0/1 |
| REG_PRESENT_CURRENT | 0x45 | 2 | s15, ~6.5 mA units |

Geometry: 4096 counts/rev, CENTER = 2048, so 1 count = 2π/4096 rad ≈ 1.5339 mrad.

## 4. counts ↔ mrad conversion, `firmware_config.h`, and its generator
Per D-001 (Option A): higher layers are symmetric/semantic; **direction and zero_offset are
absorbed at the hardware layer — which is now the firmware.** `barq_hw/BarqSystem` passes
radians straight through as mrad (`barq_system.cpp` ×1000, clamped int16); the firmware does
the whole servo-frame conversion.

**Definitions** (fix these or P2/P3 will fight): `direction d ∈ {+1,−1}` and
`zero_offset z` (rad) per servo, from `barq_description/config/robot_params.yaml` `servos:`
refined by P2's measured `artifacts/servo_calib.yaml`. `z` is defined as *the semantic joint
angle at which the servo sits at count 2048* (perfect assembly ⇒ z = 0).

```
K = 4096 / 2π  counts/rad  =  4096 / 6283.185  counts/mrad
counts(θ_mrad) = clamp( round( 2048 + d · (θ_mrad − z_mrad) · 4096 / 6283.185 ),
                        min_counts, max_counts )
θ_mrad(counts) = d · (counts − 2048) · 6283.185 / 4096 + z_mrad        (d² = 1)
```
Integer form (no float in the 500 Hz path): `2048 + (d * (θ−z) * 4096) / 6283` with int32
intermediates (max |θ−z|·4096 ≈ 9.0e6, fits) and round-half-away; error vs exact < 0.05 count.
**Limits**: `min/max_counts` are computed by the generator from `min_angle`/`max_angle` through
the same formula — when `d = −1` the images swap, so the generator emits
`min_counts = min(c(min_angle), c(max_angle))`, `max_counts = max(...)`. Velocity for STATE:
`vel_10mrad_s = s15(raw_speed) · 1.5339 · d / 10` (counts/s assumption → TBD-3); load:
`load_0p1pct = s15(raw_load) · d` (sign follows joint convention).

**`barq_firmware/src/firmware_config.h`** — generated, committed, pure constexpr (host-safe):
```cpp
// AUTO-GENERATED by diagnostics/gen_firmware_config.py — DO NOT EDIT.
// Sources: barq_description/config/robot_params.yaml + artifacts/servo_calib.yaml (@sha1s)
#pragma once
#include <stdint.h>
namespace barq {
struct ServoCfg {
  uint8_t  id;             // bus servo ID == protocol index k (ID plan)
  uint8_t  bus;            // 0..3 = FL,FR,RL,RR  (== k/3)
  int8_t   dir;            // ±1 (D-001 Option A)
  int16_t  zero_off_mrad;  // joint angle at servo count 2048
  uint16_t min_counts, max_counts;
};
inline constexpr ServoCfg kServos[12] = { /* 12 rows, servo-ID order */ };
inline constexpr uint16_t kFittedMask   = 0x0FFF; // bit k = servo k physically on the bus
inline constexpr bool     kImuFitted    = false;  // flips in P3-02
inline constexpr bool     kPowerFitted  = false;  // flips in P3-02
inline constexpr int16_t  kImuMountQuat1e4[4] = {0,0,0,10000}; // x,y,z,w (P3-02 / P1-04)
inline constexpr int8_t   kImuMountMat3[9]    = {1,0,0, 0,1,0, 0,0,1};
inline constexpr uint16_t kVbusAlertMv = 13800;   // fault bit2 (P1 default)
inline constexpr uint16_t kVbusFloorMv = 13200;   // firmware hard torque-off
}
```
**`diagnostics/gen_firmware_config.py`** spec: Python 3 + PyYAML only. Reads the two YAMLs
(calib file optional — missing servo rows fall back to robot_params values), validates 12
unique IDs 0–11, emits the header deterministically (sorted, fixed float→int rounding), embeds
source SHA1s in the banner, exits non-zero on any missing/duplicate servo.
Flags: `--out PATH` (default the firmware src path) ·
`--fitted 0,1,2` (bench subsets → kFittedMask; **unfitted servos run the v0 loopback model
in-firmware**: no bus traffic, excluded from fault bit0 — this is what makes the 3-servo bench
rig pass full-stack tests honestly) · `--bench-power` (P3-02: drops alert/floor to 11.5/10.5 V
for a 12 V bench PSU so bit2/floor logic doesn't fire without a battery).
Re-run the generator + rebuild whenever robot_params/servo_calib change; the build must fail
loudly if `firmware_config.h` is absent (it's `#include`d by `teensy_hw.cpp`, NOT by loop_core).

Expected `artifacts/servo_calib.yaml` schema (P2 owns producing it; adapt the generator if P2's
final schema differs): per servo name: `{id, zero_offset_rad, min_counts, max_counts, date}`.

## 5. Buses, serial mapping, duplex modes
One `kBusSerial[4]` array in `teensy_hw.cpp` maps bus index → `HardwareSerial&` —
**copy the assignment from P1-03's wiring table; this file deliberately does not duplicate it.**
Teensy 4.1 has 8 hardware UARTs; any 4 work, prefer ones whose pins P1-03 routed.

Both duplex modes are specced; compile-time `#define BARQ_BUS_HALF_DUPLEX 0/1` set from P1-03's
determination:
- **Half (single data wire, Teensy native)**: `serial.begin(1'000'000, SERIAL_8N1_HALF_DUPLEX)`
  (Teensyduino LPUART feature, TX pin bidirectional). **Gotcha: your own TX bytes echo into
  RX.** The driver records `tx_len` per transaction and discards exactly that many echoed bytes
  before parsing the reply. Never call blocking `flush()`; compute the TX airtime
  (`bytes × byte_µs`) and open the reply window after it.
- **Full (separate TX/RX joined by P1-03's combiner — resistor/diode-OR or buffer)**: plain
  `begin(1'000'000)`. Whether TX echoes depends on the circuit → per-build flag
  `BARQ_BUS_TX_ECHO 0/1` (measure once in G3.1 setup: send a PING, count bytes received).
  Same discard logic, gated by the flag — one parser, two configurations.

## 6. Sync-write — frame layout and the tricky lines
Per-bus target write, 3 servos, GOAL_POS only (ACC/SPEED are set once at init: `ACC=0`,
`SPEED=0` = max — the proven `cmd_sweep` streaming configuration; position commands at
100 Hz do the shaping, same as the bench tool):

```
FF FF | 0xFE | LEN | 0x83 | ADDR=0x2A | L=2 | id0 p0L p0H | id1 p1L p1H | id2 p2L p2H | chk
LEN = (L+1)·N + 4 = 3·3 + 4 = 13   →  frame total = 4 + LEN = 17 bytes
chk = (~(0xFE + LEN + 0x83 + 0x2A + L + Σ(ids + data))) & 0xFF
```
No status reply (broadcast). Builder (the ~15 lines that must be exact):
```cpp
size_t build_sync_write(uint8_t addr, uint8_t L, const uint8_t* ids,
                        const uint8_t* data /*N*L, per-servo little-endian*/, uint8_t N,
                        uint8_t* out) {
  uint8_t s = 0; size_t n = 0;
  out[n++]=0xFF; out[n++]=0xFF; out[n++]=0xFE;
  const uint8_t len = (L+1)*N + 4;
  out[n++]=len; out[n++]=0x83; out[n++]=addr; out[n++]=L;
  s = 0xFE + len + 0x83 + addr + L;
  for (uint8_t i=0;i<N;++i){ out[n++]=ids[i]; s+=ids[i];
    for (uint8_t b=0;b<L;++b){ out[n++]=data[i*L+b]; s+=data[i*L+b]; } }
  out[n++] = ~s;                      // (~sum)&0xFF — same checksum as st3215_diag.py
  return n;
}
```
**TBD-1 bench verification (do this before anything else on G3.1 day):** one servo on the bus,
send a sync-write goal ±200 counts. Servo moves + zero reply bytes ⇒ opcode confirmed; log it.
No motion ⇒ drop to ladder rung B (per-servo WRITE).

Torque enable on/off uses the same builder with `ADDR=0x28, L=1` (14-byte frame per bus).

## 7. Read strategy ladder and the timing budget
Byte time: **1 Mbaud, 8N1 → 10 bits → 10 µs/byte** (500 k → 20 µs/byte). Superloop tick =
2000 µs. `TRD` = servo reply latency (return-delay register not in the diag script → TBD-4;
budget ≤ 250 µs until measured in G3.1).

Read transaction per servo — one contiguous block `READ addr=0x38, n=14` (0x38–0x45: pos,
speed, load, volt, temp, 0x40–0x44 incl. moving, current — current/temp ride along free):
request 8 B, reply 4 + (14+2) = 20 B.

| Item @1 Mbaud | Bytes | Wire time |
|---|---|---|
| sync-write GOAL_POS ×3 | 17 | 170 µs |
| read req (n=14) | 8 | 80 µs |
| read reply | 20 | 200 µs |
| **A: per tick per bus** (write + 1 read + TRD) | 45 | **≈ 700 µs (35 % of tick)** |
| **B: per tick per bus** (write + 3 reads + 3·TRD) | 101 | ≈ 1760 µs (88 % — no margin) |

- **A (PRIMARY) — round-robin partial reads**: each tick, every bus reads ONE servo
  (`offset = tick % 3`), all 4 buses in parallel (4 independent UARTs, non-blocking).
  Every servo refreshes every 3 ticks = **6 ms → at 100 Hz STATE, all 12 values are ≤ 6 ms
  old ⇒ full refresh ≤ N = 1 STATE frame.** Per-servo sample rate 167 Hz; vel staleness ≤ 6 ms.
- **B — all 3 servos/bus/tick**: full refresh every tick (fresh vel each STATE), but 88 % bus
  duty at 1 Mbaud leaves no retry headroom, and **at 500 kbaud B needs ≈ 3.5 ms > 2 ms tick —
  impossible.** A degrades gracefully (500 k: 340+160+400+TRD ≈ 1150 µs, still fits) — that is
  why A is primary. Keep B as a compile flag for bench experiments only.
- **C (fallback) — slower full state**: reads at 50 Hz (one servo/bus every other tick), STATE
  still 100 Hz serving last-known; vel additionally derivable from position differences
  (Δpos/Δt over the 20 ms refresh, low-pass k=0.2) if REG_PRESENT_SPEED proves noisy/unscaled
  (TBD-3). Cost: vel quality; estimator unaffected (it uses pos + IMU).

Per-bus non-blocking state machine (in `bus_driver.cpp`), advanced from `TeensyHw::poll()`:
```
IDLE → (tick) stage sync-write TX → stage read request TX → AWAIT_REPLY
AWAIT_REPLY: consume echo (if half-duplex/echo), parse status frame;
  complete+chk ok → cache {pos,vel,load,volt,temp,current}, clear stale bit → IDLE
  next tick arrives first → TIMEOUT path
```
TX is fire-and-forget into the Teensy serial buffers (17+8 B ≪ buffer); never busy-wait.

## 8. Error handling, stale bitmap, fault bit0
- Checksum fail or timeout → **retry the same servo once on the next tick**; second failure →
  set its bit in `stale_mask` (uint16, lives in `BusDriver`), advance the round-robin (never
  stall the ring), **hold last-good pos/vel/load** in the cache.
- `fault bit0 = (stale_mask & kFittedMask) != 0`; clears automatically once every fitted servo
  has a fresh read. **Decision — STATE has no per-servo fault field; we do NOT smuggle a
  sentinel into the load field** (in-band magic values corrupt a real physical quantity that
  controllers/logs integrate, and the Python side would need unconditional special-casing
  forever). bit0 + firmware-side bitmap + per-bus counters is enough to stop the robot and
  diagnose on the bench; a future DIAG frame type (protocol already reserves the pattern:
  "counter in a future STATE rev", docs/06_PROTOCOL.md) exports the bitmap when needed.
- Counters per bus: `tx_frames, rx_ok, rx_chk_fail, rx_timeout, retries`. Exposed on the
  **debug CDC**: build with `-DUSB_DUAL_SERIAL` (Teensy 4.1 dual USB serial) and print a 1 Hz
  stats line on `SerialUSB1` (enumerates as a second /dev/ttyACM — verify on the Jetson).
  Protocol stream on `Serial` stays byte-clean. Never print to `Serial`.
- Boot init per fitted servo (regular WRITE/READ, replies drained): ping → read EPROM_MODE
  (≠0 ⇒ count as config error, stale-from-birth) → ACC=0 → GOAL_SPEED=0 → TORQUE_ENABLE=0.
  Absent at boot ⇒ stale-from-birth ⇒ bit0 (unless excluded by `kFittedMask`).

## 9. Torque-enable management (integrates with existing LoopCore logic)
- **ON** at the rising fresh-edge (first valid CMD inside the deadman window) — LoopCore §2
  calls `servo_torque(true)` → per-bus sync-write `0x28=1`. Anti-lurch is already handled at
  the right layer: BarqSystem refuses activation without a live STATE and starts controllers
  from the measured pose (D-020), so the first targets ≈ current pose.
- **OFF** at the deadman falling edge (and at power hard-floor, P3-02): sync-write `0x28=0`.
  Limp is the designed failure mode (§2). The 200 ms deadman constant and its fault bit3 are
  untouched — this phase only attaches real actuation to the existing fresh/deadman seam.
- Reads continue while torque is off (STATE must reflect hand-moved joints — required by
  BarqSystem activation and by G3.2's wiggle test).

## Acceptance gates
**G3.1 — single bus, 500 Hz, 60 s clean.** Bench: 3 servos of one leg, `--fitted` mask for
them only. Firmware runs its normal loop; host streams CMD at 100 Hz holding a fixed pose
(`diagnostics/state_peek.py --step`, tool spec below). After 60 s the SerialUSB1 stats show:
`tx_frames ≈ 30 000`, `rx_ok ≈ 30 000`, **`rx_chk_fail = 0`, `rx_timeout = 0`** (zero CRC
errors), and servo temps from the read block rise < 10 °C and stay < 50 °C (stable = slope
< 0.5 °C/min after minute 2). PASS = all four numbers.

**G3.2 — 4 buses concurrent, STATE integrity on the Jetson.** All fitted servos powered.
On the Jetson (no ros2): `state_peek.py --port /dev/ttyACM0`. PASS requires all of:
(a) STATE rate 100 ± 2 Hz sustained 60 s; (b) torque off, hand-wiggle each of the 12 joints —
its pos field (and only its) moves with the correct sign per `robot_params.yaml` direction;
(c) commanded step (`--step k +300`) on one joint per bus: first STATE showing ≥ 25 % of the
step arrives ≤ 30 ms after the CMD (proves **full position refresh ≤ N = 1 STATE frame**,
strategy A's bound: ≤ 6 ms data age + 10 ms frame period + servo motion); (d) fault = 0x00
while streaming, bit0 = 0; (e) per-bus counters still zero-error over 5 min.

**Tool to write for these gates — `diagnostics/state_peek.py`** (~80 lines, reuses
`barq_control.barq_protocol` Decoder/encode_cmd/decode_state):
`state_peek.py [--port /dev/ttyACM0] [--hz 2] [--csv FILE] [--step IDX MRAD]`.
Passive mode prints rate + pos[12] + temp_max + vbus + current + fault; `--csv` logs every
frame (feeds G3.9 thermal curves). `--step` first listens for STATE, streams the measured pose
as CMD at 100 Hz for 1 s (manual anti-lurch), then steps joint IDX, logging CMD/STATE
timestamps. Note: passive mode sends no CMD ⇒ bit3 set is EXPECTED there.

## Fallback ladder (switch after 2 failed attempts or 2 h on a rung, whichever first)
1. **A1**: as specced — sync-write + strategy A @ 1 Mbaud.
2. **B1 — sync-write opcode fails TBD-1**: per-servo regular WRITE of GOAL_POS
   (9 B + 6 B status + TRD each). Try status-return suppression first (status-return-level
   register — not in the diag script, TBD-5); without it, stagger: write 1 servo/bus/tick
   (targets update at 100 Hz anyway; a 3-tick spread adds ≤ 4 ms command latency, acceptable);
   reads drop to strategy C.
3. **A2/B2 — CRC errors persist @ 1 Mbaud** (after the wiring ladder in P3-03): drop bus to
   **500 kbaud**: re-write EPROM_BAUD on every servo via `st3215_diag.py --baud` (TBD-2 value
   table) + change `kBusBaud`. Timing consequence (§7): byte = 20 µs, strategy A ≈ 1150 µs/tick
   — still fits; B becomes impossible; G3.2's latency bound relaxes to ≤ 35 ms. Record in the
   research log.
4. **C — reads still flaky**: strategy C (50 Hz reads, derived vel), gates re-run with N = 2
   STATE frames refresh; note it as a standing TBD to fix before P4 gait tuning.

## Rollback
`git checkout 4ea53a0 -- barq_firmware/` restores v0 loopback; or rebuild with
`--fitted ""` (kFittedMask = 0 ⇒ all-loopback even with the new code). Keep the last-good
`firmware.hex` from `.pio/build/teensy41/` archived in `artifacts/fw/` before every flash —
re-flashing it is the 30-second rollback.

## TBD table (measure, never guess)
| # | Value | Producing procedure |
|---|---|---|
| TBD-1 | SYNC_WRITE opcode 0x83 + layout works on ST3215 | §6 bench check, G3.1 day |
| TBD-2 | EPROM_BAUD value↔baud table | datasheet + verify: write, rescan at new baud |
| TBD-3 | PRESENT_SPEED scale (counts/s assumed) | command known sweep (cmd_sweep 0.25 Hz, ±30°), compare reported vs commanded peak |
| TBD-4 | servo return delay TRD | scope or micros() delta around one READ in G3.1 |
| TBD-5 | status-return-level register addr/values | datasheet; only needed on ladder rung B1 |

## Artifacts → docs/05_RESEARCH_LOG.md
G3.1/G3.2 counter screenshots or pasted stats lines, measured TRD, chosen duplex mode + echo
flag, baud decision, `state_peek` CSVs into `artifacts/`, the generated `firmware_config.h`
committed, and a log entry: believed → measured → changed (per the standing practice).

## Escape hatch
If the Teensy-side bus driver is unworkable after the full ladder (e.g., LPUART half-duplex
proves unreliable at any baud): fall back to **Jetson-direct servo buses** — 4× Waveshare
USB-UART driver boards on the Jetson, a Python bus daemon publishing into ros2_control, Teensy
retained only for IMU/power/deadman-LED. Costs: lose 500 Hz determinism and the firmware-local
deadman torque-off (deadman must then live in the daemon — implement it FIRST), USB latency
jitter. This is a P4-blocking architecture change — write it up as a D-number before starting.
