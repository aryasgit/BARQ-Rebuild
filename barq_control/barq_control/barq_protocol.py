"""
BARQ Jetson<->Teensy binary protocol v1 — Python reference implementation.

Spec: docs/06_PROTOCOL.md. The C++ twin lives in barq_firmware/src/protocol.{h,cpp};
both are pinned to the same golden vectors (test_protocol.py generates/checks them).
Pure bytes — no ROS, no serial; usable from the diagnostics bench tools, the future
hardware-interface tests, and pytest.
"""

import struct

MAGIC = b'\xba\x51'
VERSION = 1
TYPE_CMD = 0x01
TYPE_STATE = 0x02
TYPE_PING = 0x03
TYPE_PONG = 0x83

_STATE_FMT = '<12h12h12h4h3h3hHhbB'
STATE_PAYLOAD_LEN = struct.calcsize(_STATE_FMT)   # 98
CMD_PAYLOAD_LEN = 24
MAX_PAYLOAD = 200


def crc16_ccitt(data, crc=0xFFFF):
    """Compute the CCITT-FALSE CRC16 (poly 0x1021, init 0xFFFF), bytewise."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def frame(msg_type, payload, seq=0):
    """Wrap a payload with BARQ framing: magic, header, payload, crc16."""
    body = bytes([VERSION, msg_type, seq & 0xFF, len(payload)]) + payload
    return MAGIC + body + struct.pack('<H', crc16_ccitt(body))


def encode_cmd(targets_rad, seq=0):
    """12 joint targets (rad, servo-ID order) -> CMD frame (int16 milliradians)."""
    if len(targets_rad) != 12:
        raise ValueError('need 12 targets')
    mrad = [max(-32767, min(32767, round(t * 1000.0))) for t in targets_rad]
    return frame(TYPE_CMD, struct.pack('<12h', *mrad), seq)


def encode_state(pos_rad, vel_rad_s, load_pct, quat, gyro_rad_s, accel_m_s2,
                 vbus_v, current_a, temp_max_c, fault, seq=0):
    """Pack a full STATE frame from SI values (firmware does the same scaling)."""
    payload = struct.pack(
        _STATE_FMT,
        *[round(p * 1000.0) for p in pos_rad],
        *[round(v * 100.0) for v in vel_rad_s],          # 10 mrad/s units
        *[round(ld * 10.0) for ld in load_pct],          # 0.1 % units
        *[round(q * 10000.0) for q in quat],
        *[round(g * 1000.0) for g in gyro_rad_s],
        *[round(a * 100.0) for a in accel_m_s2],         # cm/s^2
        round(vbus_v * 1000.0), round(current_a * 1000.0),
        round(temp_max_c), fault & 0xFF)
    return frame(TYPE_STATE, payload, seq)


def decode_state(payload):
    """Unpack a STATE payload into a dict of SI values."""
    if len(payload) != STATE_PAYLOAD_LEN:
        raise ValueError(f'STATE payload must be {STATE_PAYLOAD_LEN} B, got {len(payload)}')
    v = struct.unpack(_STATE_FMT, payload)
    return {
        'pos': [x / 1000.0 for x in v[0:12]],
        'vel': [x / 100.0 for x in v[12:24]],
        'load': [x / 10.0 for x in v[24:36]],
        'quat': [x / 10000.0 for x in v[36:40]],
        'gyro': [x / 1000.0 for x in v[40:43]],
        'accel': [x / 100.0 for x in v[43:46]],
        'vbus': v[46] / 1000.0,
        'current': v[47] / 1000.0,
        'temp_max': v[48],
        'fault': v[49],
    }


def decode_cmd(payload):
    """Unpack a CMD payload -> 12 target positions in rad."""
    if len(payload) != CMD_PAYLOAD_LEN:
        raise ValueError(f'CMD payload must be {CMD_PAYLOAD_LEN} B, got {len(payload)}')
    return [x / 1000.0 for x in struct.unpack('<12h', payload)]


class Decoder:
    """Resync-capable streaming frame decoder (mirrors the C++ state machine)."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        """Feed raw bytes; return a list of (type, seq, payload) for each valid frame."""
        self.buf += data
        out = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                # keep at most 1 byte (could be the first magic byte of a split frame)
                del self.buf[:max(0, len(self.buf) - 1)]
                return out
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 8:
                return out
            ver, mtype, seq, ln = self.buf[2], self.buf[3], self.buf[4], self.buf[5]
            if ver != VERSION or ln > MAX_PAYLOAD:
                del self.buf[:1]
                continue
            total = 6 + ln + 2
            if len(self.buf) < total:
                return out
            body = bytes(self.buf[2:6 + ln])
            crc = struct.unpack('<H', self.buf[6 + ln:total])[0]
            if crc16_ccitt(body) == crc:
                out.append((mtype, seq, bytes(self.buf[6:6 + ln])))
                del self.buf[:total]
            else:
                del self.buf[:1]   # corrupt: drop one byte, rescan
