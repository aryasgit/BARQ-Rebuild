# barq_hw — Stage-4 hardware interface + Teensy emulator

The "drop the stack in" package. Two artifacts:

- **`barq_hw/BarqSystem`** (ros2_control SystemInterface): 12 position-commanded joints over
  BARQ protocol v1 (docs/06_PROTOCOL.md). `write()` = one CMD frame per controller cycle
  (100 Hz, int16 mrad, servo-ID order); `read()` drains the fd through the **same C++
  Decoder the firmware uses** and serves the freshest STATE (pos+vel). Activation requires
  a live STATE so controllers start from the measured pose (anti-lurch); a stale link
  (> `state_timeout_ms`) returns ERROR and stops controllers; the firmware's 200 ms deadman
  is the safety floor below that.
- **`teensy_emulator`**: the REAL firmware superloop (`barq_firmware/src/loop_core.{h,cpp}`,
  shared verbatim with `main.cpp`) compiled for the host, serving a PTY at 500 Hz.
  `ros2 run barq_hw teensy_emulator` prints `PTY /dev/pts/N`.

## The drop-in contract
```bash
# today (zero hardware):
ros2 run barq_hw teensy_emulator        # -> PTY /dev/pts/N
ros2 launch barq_bringup real.launch.py device:=/dev/pts/N gait:=true

# hardware day (Teensy flashed with the SAME LoopCore):
ros2 launch barq_bringup real.launch.py device:=/dev/ttyACM0 gait:=true
```
Nothing else changes. Verified end-to-end by `test/integration_pty.py` (also installed:
`ros2 run barq_hw integration_pty.py`): 9 checks — Python-codec-vs-firmware bench
(PING/PONG, CMD->STATE echo at the 3 mrad quantization floor, deadman bit3) plus the full
controller_manager stack on the PTY (100 Hz /joint_states, round-trip convergence).
Full-gait rehearsal: `real.launch.py gait:=true` against the emulator walks (knee swing
~330 mrad at /joint_states 100 Hz).

## v1 scope
Joints only (12x pos cmd, 12x pos+vel state). STATE's IMU/power/fault fields are decoded
and fault!=deadman is logged, but not yet exported as ros2_control sensor interfaces —
hardware day adds an IMU broadcaster (the estimator subscribes to `/imu/data`).
