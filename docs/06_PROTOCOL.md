# BARQ Jetson <-> Teensy Binary Protocol — v1

> The interface contract between the Jetson (ros2_control hardware interface, Stage 4) and the
> Teensy 4.1 superloop (Stage 3), over USB CDC serial (full-speed; framing below is rate-agnostic).
> Reference implementations: `barq_control/barq_protocol.py` (Jetson/bench, pytest-covered) and
> `barq_firmware/src/protocol.{h,cpp}` (Teensy, Unity-tested natively). Both are pinned to the
> same golden vectors — change the protocol ONLY by changing both + the vectors together.

## Frame layout (little-endian)
| Field | Size | Value |
|---|---|---|
| magic | 2 B | `0xBA 0x51` |
| ver | 1 B | `0x01` |
| type | 1 B | message type (below) |
| seq | 1 B | rolling counter per sender |
| len | 1 B | payload length in bytes |
| payload | len B | message-specific |
| crc | 2 B | CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) over ver..payload |

Decoder is a resync-capable state machine: scan for magic, validate len <= 200, verify CRC,
deliver; on any mismatch, drop one byte and rescan (USB byte streams are not packet-aligned).

## Message types
### 0x01 CMD — Jetson -> Teensy (every control tick, also serves as the deadman heartbeat)
| Field | Type | Units |
|---|---|---|
| targets[12] | int16 | joint target position, **milliradians**, servo-ID order (robot_params) |
Payload 24 B. Teensy applies `direction`/`zero_offset` from its config (the semantic frame is
symmetric, per D-001/joint_conventions).

### 0x02 STATE — Teensy -> Jetson at 100 Hz
| Field | Type | Units |
|---|---|---|
| pos[12] | int16 | mrad |
| vel[12] | int16 | 10 mrad/s |
| load[12] | int16 | 0.1 % |
| quat[4] | int16 | x,y,z,w x 1e-4 (BNO085 SH-2 rotation vector) |
| gyro[3] | int16 | mrad/s |
| accel[3] | int16 | cm/s^2 |
| vbus | uint16 | mV (INA226) |
| current | int16 | mA (INA226) |
| temp_max | int8 | degC, hottest servo |
| fault | uint8 | bit0 servo-bus error, bit1 IMU stale, bit2 power alert, bit3 deadman tripped |
Payload 98 B -> 103 B framed -> ~10.3 kB/s at 100 Hz (trivial for USB CDC).

### 0x03 PING / 0x83 PONG — link test, payload: uint32 echo token.

## Safety semantics
- **Deadman**: no valid CMD for **200 ms** -> Teensy ramps torque off, sets fault bit3. (Mirrors
  the gait-level 1 s cmd_vel deadman — two independent layers.)
- CRC failure: frame dropped silently (counter in a future STATE rev); never applied.
- The Teensy NEVER moves joints except on a valid, fresh CMD.

## v0 firmware = loopback mode
Until servos/IMU arrive, the firmware echoes CMD targets back as STATE positions (sensors stubbed,
flagged by fault bits cleared + a build banner). This lets the Stage-4 C++ hardware interface be
developed and integration-tested against a real Teensy with zero peripherals attached.
