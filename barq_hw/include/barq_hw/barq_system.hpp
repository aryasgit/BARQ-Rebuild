// BARQ Stage-4 ros2_control hardware interface: speaks protocol v1 over a serial fd.
//
// One SystemInterface for all 12 joints. write() sends a CMD frame (int16 mrad, servo-ID
// order) every controller_manager cycle (100 Hz); read() drains the fd through the SAME
// C++ Decoder the firmware uses and serves the freshest STATE (pos + vel). The firmware's
// 200 ms deadman is the safety floor: if this process dies, torque drops.
//
// `device` is a hardware <param> in the URDF (mode:=real): /dev/ttyACM0 on the robot, the
// emulator's PTY in integration tests. No other code changes between the two — that is the
// drop-in property this package exists for.
#pragma once

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

#include "protocol.h"   // barq_firmware/src — single source of truth for the wire format

namespace barq_hw
{

class BarqSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Drain the fd through the decoder; updates pos_/vel_/fault_ from every STATE seen.
  // Returns the number of STATE frames consumed.
  int drain_rx_();
  void close_fd_();

  std::string device_;
  int state_timeout_ms_ = 300;
  int fd_ = -1;

  // All arrays in protocol slot order (servo IDs 0-11 = FL,FR,RL,RR x hip,knee,ankle).
  std::vector<double> pos_, vel_, cmd_;
  std::vector<size_t> joint_slot_;        // info_.joints index -> protocol slot

  barq::Decoder decoder_;
  uint8_t tx_seq_ = 0;
  uint8_t fault_ = 0;
  int64_t last_state_us_ = 0;             // CLOCK_MONOTONIC microseconds
  int rx_error_streak_ = 0;
};

}  // namespace barq_hw
