// Host twin of the BARQ Teensy v0 loopback firmware.
//
// Runs the EXACT firmware superloop logic (barq_firmware/src/loop_core.{h,cpp}) at 500 Hz
// against a freshly created PTY, so the full Jetson stack — controller_manager ->
// barq_hw/BarqSystem -> serial bytes -> firmware decision logic — runs with zero hardware.
// Drop-in day is then literally `device:=/dev/ttyACM0` instead of the printed PTY.
//
// Usage:  ros2 run barq_hw teensy_emulator
// Prints one line `PTY /dev/pts/N` to stdout (flushed), then serves until killed.
#include <errno.h>
#include <fcntl.h>
#include <pty.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "loop_core.h"

namespace {

constexpr uint32_t LOOP_PERIOD_US = 2000;      // 500 Hz, same as the Teensy

uint32_t now_us() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint32_t>(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
}

void tx(void* user, const uint8_t* data, size_t len) {
  const int fd = *static_cast<int*>(user);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::write(fd, data + off, len - off);
    if (n > 0) {
      off += static_cast<size_t>(n);
    } else if (errno == EAGAIN || errno == EINTR) {
      continue;                                // peer slow: PTY buffer full, retry
    } else {
      return;                                  // peer gone (EIO): drop, keep serving
    }
  }
}

}  // namespace

int main() {
  int master = -1, slave = -1;
  char name[128] = {0};
  if (openpty(&master, &slave, name, nullptr, nullptr) != 0) {
    perror("openpty");
    return 1;
  }
  // Keep `slave` open on our side: the PTY then survives the peer closing/reopening the
  // device between test phases (otherwise reads on the master start failing with EIO).
  fcntl(master, F_SETFL, O_NONBLOCK);

  printf("PTY %s\n", name);
  fflush(stdout);

  barq::LoopCore core;
  uint8_t buf[512];
  while (true) {
    const uint32_t t0 = now_us();

    ssize_t n;
    while ((n = ::read(master, buf, sizeof(buf))) > 0) {
      for (ssize_t i = 0; i < n; ++i) core.rx_byte(buf[i], t0, tx, &master);
    }
    core.tick(t0, tx, &master);

    const uint32_t elapsed = now_us() - t0;
    if (elapsed < LOOP_PERIOD_US) usleep(LOOP_PERIOD_US - elapsed);
  }
}
