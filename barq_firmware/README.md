# barq_firmware — Teensy 4.1 (Stage 3)

PlatformIO project. **v0 = LOOPBACK firmware**: full protocol + deadman, sensors/servo bus
stubbed — flash it the day the Teensy arrives and the Stage-4 ros2_control hardware interface
can be integration-tested with zero peripherals attached (commanded targets echo back as
smoothed positions at 100 Hz).

## Protocol
`docs/06_PROTOCOL.md` is the contract. `src/protocol.{h,cpp}` is the C++ codec — **pure C++,
no Arduino deps** — and is pinned byte-for-byte to the Python twin
(`barq_control/barq_protocol.py`) via shared golden vectors in both test suites.

## Build & test (works on the Jetson, no Teensy needed)
```bash
export PATH="$HOME/.local/bin:$PATH"      # platformio
pio test -e native        # protocol codec unit tests (host-compiled, Unity)
pio run  -e teensy41      # full firmware build (.pio/build/teensy41/firmware.hex)
```
Flash (when hardware exists): `pio run -e teensy41 -t upload` with the Teensy on USB.

## Superloop (loop_core.{h,cpp} — shared with the host emulator)
ALL protocol/deadman/telemetry logic lives in `LoopCore` (pure C++, caller owns clock+IO):
500 Hz loop: drain USB -> frame decoder -> CMD updates targets / PING gets PONG;
deadman (200 ms without CMD -> torque off + fault bit3; LED: solid=driven, blink=idle);
100 Hz STATE telemetry. `main.cpp` is a thin Arduino shim; `barq_hw`'s `teensy_emulator`
runs the SAME LoopCore on a PTY, so the Stage-4 interface integration-tests against real
firmware logic with zero hardware (barq_hw/test/integration_pty.py, 9/9). Hardware stubs
to fill at Stage 3 proper (inside LoopCore):
`servo_bus_*` (ST3215 sync-write over 4 UARTs — register map already proven in
`diagnostics/st3215_diag.py`), `imu_read` (BNO085 SH-2), `power_read` (INA226).
