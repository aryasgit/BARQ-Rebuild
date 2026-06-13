# P1-03 — Teensy 4.1 ↔ 4 Servo Buses: wiring, interface determination, smoke tests

> Phase P1 · verified against repo @ 4ea53a0

## Objective
Wire the Teensy 4.1 to the four Waveshare driver boards (one per leg, 3 servos each) on four
hardware serial ports, RESOLVE BY MEASUREMENT the driver board's host-side signaling (the one
genuinely unknown interface in the robot), fix the bus↔leg labeling forever, and prove every
bus with a PING from the Teensy. P3 fills `servo_bus_write_targets()` /
`servo_bus_read_state()` (`barq_firmware/src/loop_core.h`); this file delivers the verified
copper those stubs will drive.

## Prerequisites
- P1-01 gate G1.1 passed (12 V rail exists and survives a stall).
- Bench truth established: each driver board + ≥ 1 servo already answers
  `diagnostics/st3215_diag.py ping/scan` over the board's USB path (the proven path from the
  diagnostics README) — never debug Teensy wiring with an unproven board.
- Host: PlatformIO installed (P0). If benching from the Jetson: `sudo apt remove brltty`,
  user in `dialout`, then replug USB.
- USB-UART adapter (3.3 V TTL, ₹150–400) + jumper wires; multimeter; scope optional but
  decisive for the determination procedure.
- Servo bus runs at **1 Mbaud half-duplex Feetech STS** (st3215_diag.py header; the Teensy
  must end up speaking exactly the register protocol documented in that script).

## Teensy 4.1 hardware-serial pin table (public — VERIFY against the PJRC pinout card)
| Port | RX pin | TX pin | Used here | Why / conflict notes |
|---|---|---|---|---|
| Serial1 | 0 | 1 | **BUS1 = FL** | free, USB-end corner — clean routing |
| Serial2 | 7 | 8 | **BUS2 = FR** | free |
| Serial3 | 15 | 14 | spare (5th bus / debug) | free — keep unpopulated |
| Serial4 | 16 | 17 | NO | pins double as **Wire1 SCL1/SDA1** — reserved for BNO085 (P1-04) |
| Serial5 | 21 | 20 | **BUS3 = RL** | free |
| Serial6 | 25 | 24 | NO | pins double as **Wire2 SCL2/SDA2** — keep I2C expansion open |
| Serial7 | 28 | 29 | **BUS4 = RR** | free |
| Serial8 | 34 | 35 | alternate for Serial7 if needed | free |
Rationale: this selection leaves **Wire (18/19) for INA260s (P1-02), Wire1 (16/17) for the
BNO085 (P1-04), Wire2 (24/25) spare, and SPI (11/12/13 + CS 10) untouched for the BNO085
SPI fallback** — the whole P1 pin budget is conflict-free by construction. Write the final
mapping on a label on the Teensy itself.

## Bus labeling = leg mapping (fixed, matches robot_params.yaml + st3215_diag.py ID_PLAN)
```
BUS1 = Serial1 = FL : IDs 0 (FL_coxa), 1 (FL_femur), 2 (FL_tibia)
BUS2 = Serial2 = FR : IDs 3,4,5
BUS3 = Serial5 = RL : IDs 6,7,8
BUS4 = Serial7 = RR : IDs 9,10,11
```
(coxa/femur/tibia = hip/knee/ankle in older notes.) Servo IDs are ASSIGNED in P2 (one servo
at a time on the bench tool); in THIS phase a fresh servo answers at factory ID 1. Label
both ends of every bus cable `BUS1-FL` … `BUS4-RR` before first power.

## THE critical unknown: driver-board host-side signaling (resolve, don't assume)
The Waveshare driver board talks to the servos on a single-wire half-duplex Feetech bus.
What it exposes to a HOST on its TTL header is product-specific. Two hypotheses:
- **H1 — full-duplex TTL pair:** board carries TX and RX pins and does the half-duplex
  direction handling on-board. Host wiring is a normal crossed UART. (Waveshare's adapter
  products commonly do this.)
- **H2 — raw single-wire half-duplex:** the header pin IS the servo data line; the host must
  share one wire for TX and RX (and will hear its own echo).

### 15-minute determination procedure (board + one servo + USB-UART; no Teensy at risk)
1. **Visual (1 min):** photograph the host header. Distinct pins silk-screened TX and RX →
   H1 likely. A single S/DATA/SIG pin → H2. Record the silkscreen verbatim.
2. **Idle levels (2 min):** board powered at 12 V, servo attached, host header unconnected.
   DMM each signal pin to GND. Expected: idle-high ~3.3 V (or 5 V — RECORD IT; if any signal
   pin idles at 5 V, the Teensy needs to be checked for tolerance — Teensy 4.1 digital pins
   are NOT 5 V tolerant; a 1 kΩ series + 3.3 V zener or level shifter is then mandatory).
3. **Behavioral (7 min):** wire USB-UART per H1 (adapter TX → board RX, adapter RX ← board
   TX, GND–GND; 3.3 V adapter setting). Run from the host:
   `./diagnostics/st3215_diag.py --port /dev/ttyUSB0 ping 1`.
   - Reply received → **H1 confirmed.** Done.
   - No reply → move both adapter wires to the single data pin candidate: adapter TX through
     a **1 kΩ series resistor** to the pin, adapter RX directly to the same pin. Re-run ping.
     Reply (possibly preceded by an echo of the request bytes) → **H2 confirmed.**
4. **Echo telltale (3 min, confirms H2 and shapes P3 firmware):** with a scope or by adding
   a debug dump to the script's `_txrx`, check whether received data begins with an exact
   copy of the transmitted packet. Echo present → half-duplex shared line; the P3 driver
   must discard `len(tx)` echoed bytes per transaction. Record yes/no.
5. Log the outcome as a research-log entry + a decision line "DRIVER-IF: H1|H2, idle X V,
   echo Y/N". P3's UART driver design hangs off this line.

### Wiring per outcome (per bus; ×4)
```
H1 (full-duplex header)                 H2 (single-wire data pin)
Teensy TXn ----------> board RX         Teensy TXn --[1 kΩ]--+---- board DATA
Teensy RXn <---------- board TX         Teensy RXn ----------+
Teensy GND <---------> board GND        Teensy GND <--------> board GND
                                        (P3 option: Teensy LPUART half-duplex
12 V to board power terminals            mode on TXn only — Teensyduino
comes from the P1-01 star, NOT           SERIAL_HALF_DUPLEX, verify support —
through the Teensy.                      then RXn stays free and no echo handling)
```
Signal wires: 24–28 AWG, twisted with their ground return, **≤ 30 cm** Teensy→board.
The 3-pin servo daisy chains (power + data) are the Waveshare leads; keep total per-leg
chain ≤ ~50 cm until the baud-vs-length test below says otherwise.

## Common-ground rules (brownouts and ghost bytes are born here)
1. ONE ground star for power: buck −, all four board power grounds, BEC −, battery − (via
   the main harness). Servo return current flows ONLY in power-harness grounds.
2. Each bus signal ground (Teensy GND ↔ board GND, the thin wire above) parallels a power
   ground path — that is fine and required; the thin wire references the UART, the thick one
   carries amps. Never let the thin wire be the ONLY path (boards power-grounded only through
   the Teensy = melted Teensy trace on first stall).
3. Teensy ↔ Jetson share ground through USB automatically; do not add a second fat
   Teensy-to-power-star ground (loop), the signal grounds via the boards suffice.
4. Measure: with everything connected and 12 V off, DMM ohms between Teensy GND and buck −:
   expect < 1 Ω. With 12 V on and a servo sweeping: DMM mV between board GND and Teensy
   GND: expect single-digit mV (tens of mV → relocate the signal ground/star).

## Procedure
1. Determination procedure above → record H1/H2 (G1.6).
2. Wire BUS1 only, per the outcome. Triple-check 12 V never lands on a signal pin (DMM the
   header with power on before connecting the Teensy: every signal pin ≤ idle logic level).
3. Minimal smoke-test sketch (behavioral spec, ~30 lines, lives in P3's firmware doc as the
   real code; reproduce by behavior here): configure `Serial1` (or LPUART half-duplex mode)
   at **1,000,000 baud 8N1**; transmit the Feetech PING frame for ID 1
   (`FF FF 01 02 01 FB` — the exact framing/checksum is implemented in
   `diagnostics/st3215_diag.py::Bus._txrx`; port it verbatim); print received bytes hex to
   USB serial. Expected reply: `FF FF 01 02 00 FC` (status packet, error byte 0x00) within
   ~1 ms (under H2 preceded by the echo of the request — discard it).
4. Repeat for BUS2/3/4, one at a time, same single test servo if only one is unpacked —
   moving one known-good servo across buses isolates board faults from servo faults (G1.7).
5. Sustained-traffic check per bus: loop the PING (or a 6-byte position read at 0x38,
   `st3215_diag` register map) at ≥ 200 Hz for 60 s; count malformed/missing replies.
   Expected: **0 errors at 1 Mbaud** with bench-length leads (G1.8).
6. Harness-length rehearsal: re-run step 5 with the REAL chassis-length leads for the
   longest leg run. Record error counts → this is the 1 Mbaud-vs-length evidence (TBD row).

## TBD table
| ID | Value | Procedure |
|---|---|---|
| TBD-12 | driver-board host interface (H1/H2), idle level, echo | determination procedure above |
| TBD-13 | max reliable harness length @ 1 Mbaud | step 6 sweep with real lead lengths |
| TBD-14 | per-transaction servo turnaround time | scope/timestamp during step 5 (feeds P3 loop budget) |

## Acceptance gates
- **G1.6** — TBD-12 resolved by measurement and logged (hypothesis, idle voltage, echo
  behavior); if idle > 3.4 V, the level-protection measure is installed and re-verified.
- **G1.7** — Teensy PING success (correct status packet) on **each of the four buses**, same
  servo, correct bus labels physically attached.
- **G1.8** — 60 s sustained traffic per bus at 1 Mbaud with **zero** framing/checksum
  errors on bench leads; chassis-length leads pass or the baud fallback decision is logged.

## Fallback ladders
**A bus is dead (G1.7 fails on some bus):** swap-test matrix — change ONE element per trial:
| Trial | Board | Cable | Teensy port | Servo |
|---|---|---|---|---|
| 1 | suspect | known-good | suspect | known-good |
| 2 | known-good | known-good | suspect | known-good |
| 3 | suspect | known-good | known-good (Serial3 spare) | known-good |
| 4 | suspect | suspect | suspect | known-good |
Two trials isolate board-vs-port; ≤ 4 isolate any single culprit. A dead Teensy PORT →
remap to Serial3/Serial8 (table above) and update the label + P3 pin config. A dead BOARD →
that leg's board is replaced (boards owned ×4; if no spare, buy to the same product line —
₹600–1,200 class — and re-run the determination procedure on the new revision: do not
assume H-result transfers across revisions).

**1 Mbaud unstable on real harness lengths (G1.8 fails at length):** baud ladder
1 M → 500 k → 250 k → 115.2 k (ST3215 EPROM baud register 0x06 per `st3215_diag.py`; value
table verify against Feetech docs — change servo AND Teensy together, one servo at a time
on the bench tool). *Switch criteria:* > 0 errors/min at the current rung after re-checking
grounds/twisting/length once. **Timing-budget consequence (flag for P3):** the 100 Hz loop
gives each bus 10 ms. Per servo, a read+write cycle is ≈ 30–40 bytes on the wire + turnaround
(TBD-14). At 1 M (10 µs/byte) a 3-servo bus costs ~1.5–3 ms — comfortable. At 115.2 k
(86.8 µs/byte) it costs ~10–15 ms — DOES NOT FIT: P3 must then drop STATE to 50 Hz or split
reads across cycles. Record the chosen rung in robot-level config and in the research log;
1 M is the design point, anything lower is a documented debt.

## Rollback
Any bus misbehaving after wiring: fall back to the proven USB bench path
(`st3215_diag.py` direct to the board) to re-prove board+servo health, then re-approach the
Teensy wiring. Keep the USB path cabling intact through all of P1–P3; it is the permanent
lowest-layer test point (doomsday protocol §4.3).

## Artifacts → docs/05_RESEARCH_LOG.md
TBD-12/13/14 entries; the four-bus label photo; G1.7 reply dumps (hex); G1.8 error counts
per bus per baud; any swap-matrix outcome; the DRIVER-IF decision line (also mirror into
docs/02_DECISIONS.md — P3 firmware reads it).

## If this entire phase approach fails
If the Teensy cannot reliably master the boards' TTL interface at any baud (after H1/H2
resolution, level protection, ground fixes, and the swap matrix): bypass the Teensy for bus
mastering — plug all four driver boards' USB interfaces into a powered USB hub on the
Jetson and drive the Feetech protocol from the Jetson directly (the `st3215_diag.py` stack
already proves this path per-board). Consequences, accepted explicitly before switching:
the 200 ms deadman and servo I/O move into Jetson userspace (weaker realtime), the P3
firmware plan is rewritten (Teensy keeps IMU + power + fault duty only, or is dropped), and
USB latency budgets must be re-measured. This is an architecture change → docs/02_DECISIONS.md
entry with the measurements that forced it.
