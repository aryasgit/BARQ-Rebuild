#include "barq_hw/barq_system.hpp"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace barq_hw
{

namespace
{

rclcpp::Logger logger() {return rclcpp::get_logger("BarqSystem");}

int64_t mono_us()
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

// Protocol slot order = servo IDs 0-11 (robot_params.yaml): legs FL,FR,RL,RR x hip,knee,ankle.
const char * kSlotNames[12] = {
  "FL_hip_joint", "FL_knee_joint", "FL_ankle_joint",
  "FR_hip_joint", "FR_knee_joint", "FR_ankle_joint",
  "RL_hip_joint", "RL_knee_joint", "RL_ankle_joint",
  "RR_hip_joint", "RR_knee_joint", "RR_ankle_joint",
};

}  // namespace

hardware_interface::CallbackReturn BarqSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto param = [this](const std::string & key, const std::string & fallback) {
      auto it = info_.hardware_parameters.find(key);
      return it == info_.hardware_parameters.end() ? fallback : it->second;
    };
  device_ = param("device", "/dev/ttyACM0");
  state_timeout_ms_ = std::stoi(param("state_timeout_ms", "300"));

  if (info_.joints.size() != 12) {
    RCLCPP_ERROR(logger(), "expected 12 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  pos_.assign(12, 0.0);
  vel_.assign(12, 0.0);
  cmd_.assign(12, 0.0);
  joint_slot_.resize(12);
  for (size_t i = 0; i < 12; ++i) {
    const auto & j = info_.joints[i];
    size_t slot = 12;
    for (size_t s = 0; s < 12; ++s) {
      if (j.name == kSlotNames[s]) {slot = s; break;}
    }
    if (slot == 12) {
      RCLCPP_ERROR(logger(), "joint '%s' has no protocol slot", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_slot_[i] = slot;
    if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger(), "joint '%s' must have exactly one position command interface",
        j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(logger(), "BARQ system: device=%s state_timeout=%dms",
    device_.c_str(), state_timeout_ms_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BarqSystem::on_configure(const rclcpp_lifecycle::State &)
{
  fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(logger(), "cannot open %s: %s", device_.c_str(), strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }
  struct termios tio;
  if (tcgetattr(fd_, &tio) == 0) {
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, B115200);   // cosmetic on USB CDC and PTYs alike
    cfsetospeed(&tio, B115200);
    tcsetattr(fd_, TCSANOW, &tio);
    tcflush(fd_, TCIOFLUSH);
  }
  RCLCPP_INFO(logger(), "opened %s", device_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BarqSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_fd_();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BarqSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // Handshake: PING, then require one STATE so controllers start from the REAL pose
  // (anti-lurch: first written command = current measured position, not a stale default).
  const uint8_t ping_payload[4] = {0xBA, 0x51, 0x00, 0x01};
  uint8_t out[barq::MAX_FRAME];
  const size_t n = barq::make_frame(barq::TYPE_PING, tx_seq_++, ping_payload, 4, out);
  if (::write(fd_, out, n) != static_cast<ssize_t>(n)) {
    RCLCPP_WARN(logger(), "PING write incomplete on %s", device_.c_str());
  }

  const int64_t deadline = mono_us() + 1500000;
  last_state_us_ = 0;
  while (mono_us() < deadline) {
    if (drain_rx_() > 0) {break;}
    usleep(2000);
  }
  if (last_state_us_ == 0) {
    RCLCPP_ERROR(logger(), "no STATE from %s within 1.5 s — firmware not running?",
      device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (size_t s = 0; s < 12; ++s) {cmd_[s] = pos_[s];}
  RCLCPP_INFO(logger(), "firmware alive on %s (fault=0x%02x); holding measured pose",
    device_.c_str(), fault_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BarqSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop commanding; the firmware deadman (200 ms) drops torque on its own.
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BarqSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < 12; ++i) {
    const size_t s = joint_slot_[i];
    ifaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[s]);
    ifaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[s]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> BarqSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < 12; ++i) {
    ifaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_[joint_slot_[i]]);
  }
  return ifaces;
}

int BarqSystem::drain_rx_()
{
  uint8_t buf[1024];
  int states = 0;
  ssize_t n;
  while ((n = ::read(fd_, buf, sizeof(buf))) > 0) {
    for (ssize_t i = 0; i < n; ++i) {
      if (!decoder_.feed(buf[i])) {continue;}
      if (decoder_.type() == barq::TYPE_STATE &&
        decoder_.payload_len() == barq::STATE_PAYLOAD_LEN)
      {
        barq::StatePayload s;
        memcpy(&s, decoder_.payload(), sizeof(s));
        for (int k = 0; k < 12; ++k) {
          pos_[k] = s.pos_mrad[k] / 1000.0;
          vel_[k] = s.vel_10mrad_s[k] / 100.0;
        }
        fault_ = s.fault;
        last_state_us_ = mono_us();
        ++states;
      }
      // PONG and anything else: liveness only, nothing to store in v1.
    }
  }
  return states;
}

hardware_interface::return_type BarqSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  drain_rx_();
  const int64_t age_ms = (mono_us() - last_state_us_) / 1000;
  if (age_ms > state_timeout_ms_) {
    if (++rx_error_streak_ % 100 == 1) {
      RCLCPP_ERROR(logger(), "STATE stale: %lldms > %dms (device %s)",
        static_cast<long long>(age_ms), state_timeout_ms_, device_.c_str());
    }
    return hardware_interface::return_type::ERROR;
  }
  rx_error_streak_ = 0;
  if (fault_ & ~0x08) {   // deadman (bit3) is expected while idle; anything else is news
    RCLCPP_WARN_ONCE(logger(), "firmware fault flags: 0x%02x", fault_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BarqSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  barq::CmdPayload p;
  for (int k = 0; k < 12; ++k) {
    const double mrad = std::isfinite(cmd_[k]) ? cmd_[k] * 1000.0 : pos_[k] * 1000.0;
    p.targets_mrad[k] = static_cast<int16_t>(std::lround(
        std::clamp(mrad, -32767.0, 32767.0)));
  }
  uint8_t out[barq::MAX_FRAME];
  const size_t n = barq::make_frame(barq::TYPE_CMD, tx_seq_++,
    reinterpret_cast<const uint8_t *>(&p), sizeof(p), out);
  const ssize_t w = ::write(fd_, out, n);
  if (w != static_cast<ssize_t>(n) && errno != EAGAIN) {
    RCLCPP_WARN(logger(), "CMD write failed on %s: %s", device_.c_str(), strerror(errno));
  }
  return hardware_interface::return_type::OK;
}

void BarqSystem::close_fd_()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace barq_hw

PLUGINLIB_EXPORT_CLASS(barq_hw::BarqSystem, hardware_interface::SystemInterface)
