// BARQ firmware superloop logic, v0 "LOOPBACK" — pure C++, no Arduino.
//
// Shared VERBATIM by the Teensy build (main.cpp supplies Serial + micros) and the host
// PTY emulator (barq_hw/src/teensy_emulator.cpp supplies a POSIX fd + CLOCK_MONOTONIC).
// That sharing is the point: the Stage-4 hardware interface integration-tests against
// the real firmware decision logic with zero hardware attached.
//
// Caller owns the clock (`now_us`) and the byte sink (`TxFn`).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

namespace barq {

// TX byte sink (Serial.write on the Teensy, ::write on the host).
typedef void (*TxFn)(void* user, const uint8_t* data, size_t len);

class LoopCore {
 public:
  static constexpr uint32_t kStatePeriodUs = 10000;   // 100 Hz STATE telemetry
  static constexpr uint32_t kDeadmanUs = 200000;      // 200 ms without CMD -> torque off

  // Feed one received byte through the frame decoder. CMD updates targets and feeds the
  // deadman; PING is answered with PONG through `tx`.
  void rx_byte(uint8_t b, uint32_t now_us, TxFn tx, void* user);

  // Call every superloop iteration: deadman evaluation, (stubbed) actuation, 100 Hz STATE.
  // Returns true while driven (fresh CMD inside the deadman window) — LED logic upstream.
  bool tick(uint32_t now_us, TxFn tx, void* user);

  // Loopback servo model, exposed for tests: first-order tracking toward targets.
  const int16_t* positions_mrad() const { return positions_mrad_; }

 private:
  void send_state(uint32_t now_us, TxFn tx, void* user);

  // ── Stage-3 hardware stubs (replaced when parts arrive) ──────────────────
  void servo_bus_write_targets(const int16_t* /*mrad*/) { /* ST3215 sync-write x4 UARTs */ }
  void servo_bus_read_state(int16_t* pos, int16_t* vel, int16_t* load);
  void imu_read(int16_t* quat_1e4, int16_t* gyro, int16_t* accel);
  void power_read(uint16_t* vbus_mv, int16_t* current_ma);

  Decoder decoder_;
  int16_t targets_mrad_[12] = {0};
  int16_t positions_mrad_[12] = {0};            // loopback: tracks targets
  bool have_cmd_ = false;
  uint32_t last_cmd_us_ = 0;
  uint32_t last_state_us_ = 0;
  uint8_t tx_seq_ = 0;
};

}  // namespace barq
