// BARQ Teensy 4.1 firmware - Stage 3, v0 "LOOPBACK".
//
// Thin Arduino shim around loop_core.{h,cpp} — ALL protocol/deadman/telemetry logic lives
// there, shared byte-for-byte with the host PTY emulator (barq_hw teensy_emulator) that the
// Stage-4 hardware interface integration-tests against. Servo bus / BNO085 / INA226 are
// stubbed inside LoopCore until parts arrive.
//
// Build: pio run -e teensy41      Tests (host, no Teensy): pio test -e native
#include <Arduino.h>

#include "loop_core.h"

namespace {

constexpr uint32_t LOOP_PERIOD_US = 2000;      // 500 Hz superloop

barq::LoopCore core;

void tx(void*, const uint8_t* data, size_t len) {
  Serial.write(data, len);
}

}  // namespace

void setup() {
  Serial.begin(115200);                        // USB CDC: baud is cosmetic
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  const uint32_t now = micros();

  while (Serial.available() > 0) {
    core.rx_byte(static_cast<uint8_t>(Serial.read()), now, tx, nullptr);
  }

  const bool driven = core.tick(now, tx, nullptr);
  digitalWrite(LED_BUILTIN, driven ? HIGH : (now / 250000) % 2);   // solid=driven, blink=idle

  // Cooperative pacing (placeholder for the 500 Hz hard loop once the servo bus is real).
  const uint32_t elapsed = micros() - now;
  if (elapsed < LOOP_PERIOD_US) delayMicroseconds(LOOP_PERIOD_US - elapsed);
}
