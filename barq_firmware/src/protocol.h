// BARQ Jetson<->Teensy binary protocol v1 (spec: docs/06_PROTOCOL.md).
// Twin of barq_control/barq_protocol.py - pinned by shared golden vectors.
// Pure C++ (no Arduino deps) so it unit-tests natively on any host.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace barq {

constexpr uint8_t MAGIC0 = 0xBA, MAGIC1 = 0x51;
constexpr uint8_t VERSION = 0x01;
constexpr uint8_t TYPE_CMD = 0x01, TYPE_STATE = 0x02, TYPE_PING = 0x03, TYPE_PONG = 0x83;
constexpr size_t CMD_PAYLOAD_LEN = 24;
constexpr size_t STATE_PAYLOAD_LEN = 98;
constexpr size_t MAX_PAYLOAD = 200;
constexpr size_t MAX_FRAME = 6 + MAX_PAYLOAD + 2;

uint16_t crc16_ccitt(const uint8_t* data, size_t n, uint16_t crc = 0xFFFF);

// Build a frame around `payload`; returns total frame length written to `out`
// (caller provides >= 8 + len bytes). Returns 0 if len > MAX_PAYLOAD.
size_t make_frame(uint8_t type, uint8_t seq, const uint8_t* payload, size_t len, uint8_t* out);

#pragma pack(push, 1)
struct CmdPayload {            // 0x01, Jetson -> Teensy
  int16_t targets_mrad[12];
};
struct StatePayload {          // 0x02, Teensy -> Jetson @ 100 Hz
  int16_t pos_mrad[12];
  int16_t vel_10mrad_s[12];
  int16_t load_0p1pct[12];
  int16_t quat_1e4[4];
  int16_t gyro_mrad_s[3];
  int16_t accel_cm_s2[3];
  uint16_t vbus_mv;
  int16_t current_ma;
  int8_t temp_max_c;
  uint8_t fault;
};
#pragma pack(pop)
static_assert(sizeof(CmdPayload) == CMD_PAYLOAD_LEN, "CMD layout");
static_assert(sizeof(StatePayload) == STATE_PAYLOAD_LEN, "STATE layout");

// Resync-capable streaming decoder (mirror of the Python Decoder).
class Decoder {
 public:
  // Feed one byte; returns true when a complete valid frame is available via
  // type()/seq()/payload()/payload_len(). The frame stays valid until the next feed().
  bool feed(uint8_t b);
  uint8_t type() const { return type_; }
  uint8_t seq() const { return seq_; }
  const uint8_t* payload() const { return frame_; }
  size_t payload_len() const { return len_; }

 private:
  uint8_t buf_[MAX_FRAME];
  uint8_t frame_[MAX_PAYLOAD];   // last valid payload (copied out, survives further feeds)
  size_t have_ = 0;
  uint8_t type_ = 0, seq_ = 0;
  size_t len_ = 0;
  void resync_();
};

}  // namespace barq
