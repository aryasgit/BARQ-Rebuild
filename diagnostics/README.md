# BARQ Diagnostics — ST3215 bench tools

Pre-assembly bench station for the Waveshare ST3215 serial bus servos. **No Teensy, no ROS
needed** — a Waveshare Serial Bus Servo Driver board on USB is enough, so every servo can be
ID'd, calibrated, and proven healthy *before* it's screwed into a leg.

## Status of the comms stack (honest picture)
- The Teensy firmware (Stage 3) and the C++ ros2_control hardware interface (Stage 4) are **not
  written yet** — nothing in the repo drove a real servo before this tool.
- `st3215_diag.py` is the first hardware-facing artifact. It implements the Feetech STS register
  protocol (1 Mbps half-duplex TTL) directly over pyserial. The Teensy firmware will later speak
  the **same register map** (documented in the script) over its 4 UARTs.

## Hardware hookup (bench)
1. Waveshare driver board → USB to the Jetson (or your Mac). Appears as `/dev/ttyACM0` /
   `/dev/ttyUSB0` (Mac: `/dev/tty.usbserial-*`). The tool auto-detects, or pass `--port`.
2. Power the board from its **own 7.4 V class supply** (ST3215 rated range); never from USB.
3. Servos daisy-chain on the 3-pin bus — final robot wiring is **one driver board per leg,
   3 servos chained** (4 buses total, matching the Teensy's 4 UARTs later).
4. `pip3 install pyserial` if missing. On the Jetson you can also run inside the container:
   `docker run -it --rm --device /dev/ttyACM0 -v ~/barq_ws:/root/barq_ws barq:dev`.

## Bench procedure per servo (~2 min each, do all 12)
```bash
./st3215_diag.py plan                 # the ID map (matches robot_params.yaml servos:)
# connect ONE new servo (factory ID is 1):
./st3215_diag.py scan                 # confirm exactly one servo, note its ID
./st3215_diag.py set-id 1 <ID>        # assign its BARQ ID (0..11) - LABEL THE SERVO
./st3215_diag.py status <ID>          # volts in range? temp sane? mode 0?
./st3215_diag.py calibrate-mid <ID>   # hand-position at mechanical middle -> becomes 2048
./st3215_diag.py sweep <ID>           # sine sweep: watch tracking error, load, temperature
./st3215_diag.py torque <ID> off
```
A healthy servo: scan finds it instantly, sweep tracking error stays small and smooth (lag while
moving is normal), no grinding, temperature stable, load symmetric both directions.

Then per assembled leg (3 chained): `scan` shows all 3 IDs; `monitor` each joint while moving the
leg by hand (torque off) — positions must change smoothly with no dropouts.

## Calibration data flow
- `calibrate-mid` makes the *mechanical* middle read 2048 — do this BEFORE assembly.
- After assembly, the small residual between the assembled-neutral pose and 2048 goes into
  `robot_params.yaml` → `servos: zero_offset` (Stage 4 uses it); direction flips go into
  `direction` there too. Bench tool always works in raw servo frame.

## Safety
- No horns/linkages during first power-up and sweeps.
- IDs must be unique per bus — assign with only one servo connected.
- `torque off` before handling. Watch temperature in `monitor` during long tests.
