#include "protocol.h"

#include <string.h>

namespace barq {

uint16_t crc16_ccitt(const uint8_t* data, size_t n, uint16_t crc) {
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

size_t make_frame(uint8_t type, uint8_t seq, const uint8_t* payload, size_t len, uint8_t* out) {
  if (len > MAX_PAYLOAD) return 0;
  out[0] = MAGIC0;
  out[1] = MAGIC1;
  out[2] = VERSION;
  out[3] = type;
  out[4] = seq;
  out[5] = static_cast<uint8_t>(len);
  if (len) memcpy(out + 6, payload, len);
  const uint16_t crc = crc16_ccitt(out + 2, 4 + len);
  out[6 + len] = static_cast<uint8_t>(crc & 0xFF);         // little-endian
  out[7 + len] = static_cast<uint8_t>(crc >> 8);
  return 8 + len;
}

void Decoder::resync_() {
  // Drop the first byte and shift; cheap because frames are small and corruption is rare.
  if (have_ > 1) memmove(buf_, buf_ + 1, have_ - 1);
  have_ = (have_ > 0) ? have_ - 1 : 0;
}

bool Decoder::feed(uint8_t b) {
  if (have_ >= MAX_FRAME) resync_();
  buf_[have_++] = b;

  while (true) {
    // Hunt for magic at the buffer head.
    while (have_ >= 2 && !(buf_[0] == MAGIC0 && buf_[1] == MAGIC1)) resync_();
    if (have_ < 6) return false;

    const uint8_t ver = buf_[2];
    const size_t len = buf_[5];
    if (ver != VERSION || len > MAX_PAYLOAD) {
      resync_();
      continue;
    }
    const size_t total = 6 + len + 2;
    if (have_ < total) return false;

    const uint16_t want = static_cast<uint16_t>(buf_[6 + len]) |
                          (static_cast<uint16_t>(buf_[7 + len]) << 8);
    if (crc16_ccitt(buf_ + 2, 4 + len) == want) {
      type_ = buf_[3];
      seq_ = buf_[4];
      len_ = len;
      if (len) memcpy(frame_, buf_ + 6, len);   // copy out: survives subsequent feeds
      have_ -= total;
      if (have_) memmove(buf_, buf_ + total, have_);
      return true;
    }
    resync_();
  }
}

}  // namespace barq
