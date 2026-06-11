"""Protocol tests: round-trips, corruption rejection, resync, and the C++ golden vectors."""

from barq_control.barq_protocol import (crc16_ccitt, decode_cmd, decode_state, Decoder,
                                        encode_cmd, encode_state, frame, TYPE_CMD,
                                        TYPE_PING)
import pytest


def test_crc16_known_vector():
    """CRC16-CCITT-FALSE of ascii '123456789' is the classic 0x29B1."""
    assert crc16_ccitt(b'123456789') == 0x29B1


def test_cmd_roundtrip():
    """Encode 12 targets, stream-decode, recover values to milliradian resolution."""
    targets = [0.0, 1.047531, -1.928768, 0.1, -0.785, 2.2,
               -2.2, 0.5, 1.57, -0.001, 0.011, 0.9]
    raw = encode_cmd(targets, seq=42)
    frames = Decoder().feed(raw)
    assert len(frames) == 1
    mtype, seq, payload = frames[0]
    assert (mtype, seq) == (TYPE_CMD, 42)
    assert decode_cmd(payload) == pytest.approx(targets, abs=5.1e-4)


def test_state_roundtrip():
    """Full STATE frame survives encode->stream->decode at field resolutions."""
    raw = encode_state(
        pos_rad=[0.1 * i for i in range(12)],
        vel_rad_s=[0.05 * i for i in range(12)],
        load_pct=[2.5 * i for i in range(12)],
        quat=[0.0, 0.0, 0.7071, 0.7071],
        gyro_rad_s=[0.1, -0.2, 0.3],
        accel_m_s2=[0.0, 0.0, 9.81],
        vbus_v=7.4, current_a=1.25, temp_max_c=41, fault=0b0101, seq=7)
    frames = Decoder().feed(raw)
    assert len(frames) == 1
    s = decode_state(frames[0][2])
    assert s['pos'][3] == pytest.approx(0.3, abs=1e-3)
    assert s['quat'][2] == pytest.approx(0.7071, abs=1e-4)
    assert s['accel'][2] == pytest.approx(9.81, abs=1e-2)
    assert s['vbus'] == pytest.approx(7.4, abs=1e-3)
    assert s['temp_max'] == 41
    assert s['fault'] == 0b0101


def test_corrupt_frame_rejected_then_resync():
    """A flipped byte kills exactly that frame; the next frame still decodes."""
    good = encode_cmd([0.1] * 12, seq=1)
    bad = bytearray(encode_cmd([0.2] * 12, seq=2))
    bad[10] ^= 0xFF
    frames = Decoder().feed(bytes(bad) + good)
    assert len(frames) == 1
    assert frames[0][1] == 1


def test_stream_split_across_feeds():
    """Frames split at arbitrary byte boundaries reassemble (USB is a byte stream)."""
    raw = encode_cmd([0.3] * 12, seq=9) + frame(TYPE_PING, b'\x01\x02\x03\x04', seq=10)
    d = Decoder()
    got = []
    for b in raw:
        got += d.feed(bytes([b]))
    assert [g[0] for g in got] == [TYPE_CMD, TYPE_PING]


def test_golden_vectors_for_cpp():
    """
    Pin the exact bytes the C++ Unity tests assert against (cross-language contract).

    Regenerate ONLY together with barq_firmware/test/test_protocol/main.cpp.
    """
    cmd = encode_cmd([0.0, 0.5, -0.5, 1.0, -1.0, 1.5, -1.5, 2.0, -2.0, 0.001, -0.001, 2.2],
                     seq=0x2A)
    assert cmd.hex() == ('ba5101012a180000f4010cfee80318fcdc05'
                         '24fad00730f80100ffff98084fc5')
    ping = frame(TYPE_PING, b'\xde\xad\xbe\xef', seq=5)
    assert ping.hex() == 'ba5101030504deadbeef3fa0'
