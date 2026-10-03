// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rh56f1_hardware/backend.hpp"

namespace rh56f1_hardware {

// Canonical actuator order the backend and transport use. Each is bound to a
// URDF joint by name through the actuator_<name> hardware parameters, never by
// declaration order.
inline constexpr std::array<const char*, kActuatorCount> kActuatorKeys = {
    "thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "pinky_1"};

// ros2_control SystemInterface for one RH56F1 hand.
//
// Lifecycle contract:
//  - on_configure(): opens the transport and starts the I/O worker, which only
//    reads state. Nothing is ever written before activation.
//  - on_activate(): the explicit arming step. Refused (FAILURE, component
//    stays inactive) unless the component has been inactive for
//    min_inactive_sec_before_activate with fresh state, so controller_manager's
//    startup auto-activation cannot arm it. Seeds the command from the
//    measured position and clears latched faults.
//  - Faults (stale state, transport error, device error, non-finite command)
//    latch: writes stop and read()/write() return ERROR until the component is
//    deactivated and explicitly activated again.
class Rh56f1HW : public hardware_interface::SystemInterface {
 public:
  ~Rh56f1HW() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State& previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  bool parse_config(const hardware_interface::HardwareInfo& info, BackendConfig& config);
  std::unique_ptr<Rh56f1Transport> make_transport(const hardware_interface::HardwareInfo& info,
                                                  bool& ok) const;
  void start_worker();
  void stop_worker();
  void report_clamps(const BackendSnapshot& snapshot);

  std::string side_;
  std::string transport_kind_;
  std::string port_;
  int baudrate_ = 0;
  int device_id_ = 0;
  double worker_period_sec_ = 0.0;

  std::unique_ptr<Rh56f1Backend> backend_;
  std::thread worker_;
  std::atomic<bool> worker_running_{false};

  // Joint names in kActuatorKeys order.
  std::array<std::string, kActuatorCount> joint_names_{};

  ActuatorTargets pos_commands_{};
  ActuatorTargets pos_states_{};
  std::array<double, kActuatorCount> vel_states_{};
  std::array<double, kActuatorCount> effort_states_{};
  std::array<double, kActuatorCount> current_states_{};
  std::array<double, kActuatorCount> temperature_states_{};
  std::array<double, kActuatorCount> comm_ok_states_{};
  std::array<double, kActuatorCount> fault_states_{};
  std::array<double, kActuatorCount> clamped_states_{};
  std::array<double, kActuatorCount> is_mock_states_{};
  // Device writes since configure (same value on each joint of the hand).
  std::array<double, kActuatorCount> writes_states_{};

  bool armed_ = false;
  BackendSnapshot last_snapshot_{};
  uint64_t reported_clamp_events_ = 0;
  uint32_t reported_faults_ = kFaultNone;
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
};

}  // namespace rh56f1_hardware
