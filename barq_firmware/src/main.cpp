// BARQ Teensy 4.1 firmware - Stage 3, v0 "LOOPBACK".
//
// Superloop skeleton with the full protocol + deadman in place; servo bus / BNO085 / INA226
// are stubbed (parts not yet delivered). In loopback mode the Teensy echoes commanded joint
// targets back as measured positions at 100 Hz, so the Stage-4 ros2_control hardware
// interface can be integration-tested against REAL hardware with zero peripherals attached.
//
// Build: pio run -e teensy41      Tests (host, no Teensy): pio test -e native
#include <Arduino.h>

#include "protocol.h"

namespace {

constexpr uint32_t STATE_PERIOD_US = 10000;    // 100 Hz STATE
constexpr uint32_t LOOP_PERIOD_US = 2000;      // 500 Hz superloop
constexpr uint32_t DEADMAN_US = 200000;        // 200 ms without CMD -> torque off (fault bit3)

barq::Decoder decoder;
int16_t targets_mrad[12] = {0};
int16_t positions_mrad[12] = {0};              // loopback: tracks targets
bool have_cmd = false;
uint32_t last_cmd_us = 0;
uint32_t last_state_us = 0;
uint8_t tx_seq = 0;

// ── Stage-3 hardware stubs (replaced when parts arrive) ─────────────────────
void servo_bus_write_targets(const int16_t* /*mrad*/) { /* TODO: ST3215 sync-write x4 UARTs */ }
void servo_bus_read_state(int16_t* pos, int16_t* /*vel*/, int16_t* /*load*/) {
  // Loopback: first-order tracking toward targets (mimics a position servo).
  for (int i = 0; i < 12; ++i) pos[i] += static_cast<int16_t>((targets_mrad[i] - pos[i]) / 4);
}
void imu_read(int16_t* quat_1e4, int16_t* gyro, int16_t* accel) {
  quat_1e4[0] = 0; quat_1e4[1] = 0; quat_1e4[2] = 0; quat_1e4[3] = 10000;  // identity
  gyro[0] = gyro[1] = gyro[2] = 0;
  accel[0] = accel[1] = 0; accel[2] = 981;     // 9.81 m/s^2 in cm/s^2
}
void power_read(uint16_t* vbus_mv, int16_t* current_ma) { *vbus_mv = 7400; *current_ma = 0; }

void send_state(uint32_t now_us) {
  barq::StatePayload s = {};
  servo_bus_read_state(positions_mrad, s.vel_10mrad_s, s.load_0p1pct);
  memcpy(s.pos_mrad, positions_mrad, sizeof(positions_mrad));
  imu_read(s.quat_1e4, s.gyro_mrad_s, s.accel_cm_s2);
  power_read(&s.vbus_mv, &s.current_ma);
  s.temp_max_c = 25;
  const bool deadman = have_cmd && (now_us - last_cmd_us > DEADMAN_US);
  s.fault = deadman ? 0x08 : 0x00;

  uint8_t out[barq::MAX_FRAME];
  const size_t n = barq::make_frame(barq::TYPE_STATE, tx_seq++,
                                    reinterpret_cast<uint8_t*>(&s), sizeof(s), out);
  Serial.write(out, n);
}

}  // namespace

void setup() {
  Serial.begin(115200);                        // USB CDC: baud is cosmetic
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  const uint32_t now = micros();

  // RX: drain the USB buffer through the frame decoder.
  while (Serial.available() > 0) {
    if (!decoder.feed(static_cast<uint8_t>(Serial.read()))) continue;
    if (decoder.type() == barq::TYPE_CMD &&
        decoder.payload_len() == barq::CMD_PAYLOAD_LEN) {
      memcpy(targets_mrad, decoder.payload(), sizeof(targets_mrad));
      have_cmd = true;
      last_cmd_us = now;
    } else if (decoder.type() == barq::TYPE_PING) {
      uint8_t out[16];
      const size_t n = barq::make_frame(barq::TYPE_PONG, decoder.seq(),
                                        decoder.payload(), decoder.payload_len(), out);
      Serial.write(out, n);
    }
  }

  // Deadman + actuation (stubbed): only drive on fresh commands.
  const bool fresh = have_cmd && (now - last_cmd_us <= DEADMAN_US);
  if (fresh) servo_bus_write_targets(targets_mrad);
  digitalWrite(LED_BUILTIN, fresh ? HIGH : (now / 250000) % 2);   // solid=driven, blink=idle

  // 100 Hz telemetry.
  if (now - last_state_us >= STATE_PERIOD_US) {
    last_state_us = now;
    send_state(now);
  }

  // Cooperative pacing (placeholder for the 500 Hz hard loop once the servo bus is real).
  const uint32_t elapsed = micros() - now;
  if (elapsed < LOOP_PERIOD_US) delayMicroseconds(LOOP_PERIOD_US - elapsed);
}
