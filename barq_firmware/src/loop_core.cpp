#include "loop_core.h"

#include <string.h>

namespace barq {

void LoopCore::servo_bus_read_state(int16_t* pos, int16_t* /*vel*/, int16_t* /*load*/) {
  // Loopback: first-order tracking toward targets (mimics a position servo).
  for (int i = 0; i < 12; ++i) {
    positions_mrad_[i] += static_cast<int16_t>((targets_mrad_[i] - positions_mrad_[i]) / 4);
  }
  memcpy(pos, positions_mrad_, sizeof(positions_mrad_));
}

void LoopCore::imu_read(int16_t* quat_1e4, int16_t* gyro, int16_t* accel) {
  quat_1e4[0] = 0; quat_1e4[1] = 0; quat_1e4[2] = 0; quat_1e4[3] = 10000;  // identity
  gyro[0] = gyro[1] = gyro[2] = 0;
  accel[0] = accel[1] = 0; accel[2] = 981;      // 9.81 m/s^2 in cm/s^2
}

void LoopCore::power_read(uint16_t* vbus_mv, int16_t* current_ma) {
  *vbus_mv = 7400;
  *current_ma = 0;
}

void LoopCore::rx_byte(uint8_t b, uint32_t now_us, TxFn tx, void* user) {
  if (!decoder_.feed(b)) return;
  if (decoder_.type() == TYPE_CMD && decoder_.payload_len() == CMD_PAYLOAD_LEN) {
    memcpy(targets_mrad_, decoder_.payload(), sizeof(targets_mrad_));
    have_cmd_ = true;
    last_cmd_us_ = now_us;
  } else if (decoder_.type() == TYPE_PING) {
    uint8_t out[MAX_FRAME];
    const size_t n = make_frame(TYPE_PONG, decoder_.seq(),
                                decoder_.payload(), decoder_.payload_len(), out);
    tx(user, out, n);
  }
}

bool LoopCore::tick(uint32_t now_us, TxFn tx, void* user) {
  const bool fresh = have_cmd_ && (now_us - last_cmd_us_ <= kDeadmanUs);
  if (fresh) servo_bus_write_targets(targets_mrad_);

  if (now_us - last_state_us_ >= kStatePeriodUs) {
    last_state_us_ = now_us;
    send_state(now_us, tx, user);
  }
  return fresh;
}

void LoopCore::send_state(uint32_t now_us, TxFn tx, void* user) {
  StatePayload s = {};
  servo_bus_read_state(s.pos_mrad, s.vel_10mrad_s, s.load_0p1pct);
  imu_read(s.quat_1e4, s.gyro_mrad_s, s.accel_cm_s2);
  power_read(&s.vbus_mv, &s.current_ma);
  s.temp_max_c = 25;
  const bool deadman = have_cmd_ && (now_us - last_cmd_us_ > kDeadmanUs);
  s.fault = deadman ? 0x08 : 0x00;

  uint8_t out[MAX_FRAME];
  const size_t n = make_frame(TYPE_STATE, tx_seq_++,
                              reinterpret_cast<uint8_t*>(&s), sizeof(s), out);
  tx(user, out, n);
}

}  // namespace barq
