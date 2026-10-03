// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "openarm_hardware/openarm_simple_hardware.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <openarm/canbus/can_device.hpp>
#include <openarm/canbus/can_device_collection.hpp>
#include <stdexcept>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

namespace {
double steady_now_sec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// Margin a measured position may sit outside its limit and still be accepted
// as droop against a stop; same value and reasoning as robot_control's
// SEED_SLACK_RAD (src/robot_control/cli.py).
constexpr double kMeasuredPositionMarginRad = 0.05;
// The vendor's own motor-status check waits up to 500 ms for first replies
// (openarm_can setup/cli/commands/motor_status_commands.cpp).
constexpr int kFreshStateTimeoutMs = 500;
}  // namespace

// Replaces a motor's entry in the CAN receive-dispatch map with a wrapper
// that forwards every frame to the original device and counts it. Sending is
// unaffected: the DM device collection keeps its own device list.
class FrameCountingDevice : public openarm::canbus::CANDevice {
 public:
  explicit FrameCountingDevice(std::shared_ptr<openarm::canbus::CANDevice> inner)
      : CANDevice(inner->get_send_can_id(), inner->get_recv_can_id(),
                  inner->get_recv_can_mask(), inner->is_fd_enabled()),
        inner_(std::move(inner)) {}

  void callback(const can_frame& frame) override {
    inner_->callback(frame);
    count_.fetch_add(1, std::memory_order_relaxed);
  }
  void callback(const canfd_frame& frame) override {
    inner_->callback(frame);
    count_.fetch_add(1, std::memory_order_relaxed);
  }
  uint64_t count() const { return count_.load(std::memory_order_relaxed); }

 private:
  std::shared_ptr<openarm::canbus::CANDevice> inner_;
  std::atomic<uint64_t> count_{0};
};

OpenArmHW::OpenArmHW() = default;

bool OpenArmHW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  auto bool_param = [&info](const std::string& key, bool fallback,
                            bool& out) -> bool {
    auto found = info.hardware_parameters.find(key);
    if (found == info.hardware_parameters.end()) {
      out = fallback;
      return true;
    }
    const auto parsed = startup_safety::parse_bool(found->second);
    if (!parsed) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "hardware parameter '%s'='%s' is not one of "
                   "true/1/yes/false/0/no",
                   key.c_str(), found->second.c_str());
      return false;
    }
    out = *parsed;
    return true;
  };
  auto double_param = [&info](const std::string& key, double fallback,
                              double& out) -> bool {
    auto found = info.hardware_parameters.find(key);
    if (found == info.hardware_parameters.end()) {
      out = fallback;
      return true;
    }
    try {
      size_t used = 0;
      out = std::stod(found->second, &used);
      if (used != found->second.size() || !std::isfinite(out)) throw std::invalid_argument(key);
    } catch (const std::exception&) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "hardware parameter '%s'='%s' is not a finite number",
                   key.c_str(), found->second.c_str());
      return false;
    }
    return true;
  };

  // Gripper and CAN-FD default to true for V10.
  if (!bool_param("hand", true, hand_) || !bool_param("can_fd", true, can_fd_)) {
    return false;
  }

  // Parse control gains
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    it = info.hardware_parameters.find("kp" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kp_[i - 1] = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd" + std::to_string(i));
    if (it != info.hardware_parameters.end()) {
      kd_[i - 1] = std::stod(it->second);
    }
  }
  if (!bool_param("auto_return_to_zero", true, auto_return_to_zero_) ||
      !bool_param("verify_state_before_enable", false, verify_state_before_enable_) ||
      !double_param("min_inactive_sec_before_activate", 0.0,
                    min_inactive_sec_before_activate_) ||
      !double_param("state_stale_timeout_sec", 0.0, state_stale_timeout_sec_)) {
    return false;
  }

  // Parse ee_type (default: parallel_link for v10)
  it = info.hardware_parameters.find("ee_type");
  ee_type_ =
      (it != info.hardware_parameters.end()) ? it->second : "parallel_link";
  if (hand_) {
    it = info.hardware_parameters.find("kp_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kp_ = std::stod(it->second);
    }
    it = info.hardware_parameters.find("kd_hand");
    if (it != info.hardware_parameters.end()) {
      gripper_kd_ = std::stod(it->second);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, "
              "auto_return_to_zero=%s, min_inactive_sec_before_activate=%.3f, "
              "verify_state_before_enable=%s, state_stale_timeout_sec=%.3f",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              auto_return_to_zero_ ? "enabled" : "disabled",
              min_inactive_sec_before_activate_,
              verify_state_before_enable_ ? "true" : "false",
              state_stale_timeout_sec_);
  return true;
}

void OpenArmHW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArmHW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Optional measured-position bounds: a joint's position command interface
  // min/max, when the description declares them. The stock description does
  // not, and then only finiteness is checked.
  position_bounds_.assign(joint_names_.size(), startup_safety::Bounds{});
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    for (const auto& joint : info.joints) {
      if (joint.name != joint_names_[i]) continue;
      for (const auto& command : joint.command_interfaces) {
        if (command.name != hardware_interface::HW_IF_POSITION) continue;
        try {
          if (!command.min.empty()) position_bounds_[i].lower = std::stod(command.min);
          if (!command.max.empty()) position_bounds_[i].upper = std::stod(command.max);
        } catch (const std::exception&) {
          RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                       "joint %s: position min/max '%s'/'%s' are not numbers",
                       joint.name.c_str(), command.min.c_str(), command.max.c_str());
          return CallbackReturn::ERROR;
        }
      }
    }
    if (!(position_bounds_[i].lower < position_bounds_[i].upper)) {
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "joint %s: position min must be below max", joint_names_[i].c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Initialize OpenArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "Initializing OpenArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Initializing gripper...");
    openarm_->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }
  install_frame_counters();

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);
  temp_rotor_states_.resize(total_joints, 0.0);
  temp_mos_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  active_ = false;
  configured_at_sec_ = steady_now_sec();
  return CallbackReturn::SUCCESS;
}

void OpenArmHW::install_frame_counters() {
  auto& collection = openarm_->get_master_can_device_collection();
  std::vector<uint32_t> recv_ids;
  for (const auto& motor : openarm_->get_arm().get_motors()) {
    recv_ids.push_back(motor.get_recv_can_id());
  }
  if (hand_) {
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
      recv_ids.push_back(motor.get_recv_can_id());
    }
  }
  frame_counters_.clear();
  for (uint32_t id : recv_ids) {
    const auto& devices = collection.get_devices();
    auto found = devices.find(id);
    if (found == devices.end()) {
      // Left null: its count never advances, so activation fails closed.
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                   "no CAN device registered for recv id 0x%x; activation "
                   "will be refused",
                   id);
      frame_counters_.push_back(nullptr);
      continue;
    }
    auto counter = std::make_shared<FrameCountingDevice>(found->second);
    collection.add_device(counter);
    frame_counters_.push_back(counter);
  }
}

std::vector<uint64_t> OpenArmHW::frame_counts() const {
  std::vector<uint64_t> counts;
  counts.reserve(frame_counters_.size());
  for (const auto& counter : frame_counters_) {
    counts.push_back(counter ? counter->count() : 0);
  }
  return counts;
}

std::vector<double> OpenArmHW::motor_positions() const {
  std::vector<double> positions;
  for (const auto& motor : openarm_->get_arm().get_motors()) {
    positions.push_back(motor.get_position());
  }
  if (hand_) {
    for (const auto& motor : openarm_->get_gripper().get_motors()) {
      positions.push_back(
          motor_radians_to_joint(motor.get_position()));
    }
  }
  return positions;
}

bool OpenArmHW::await_fresh_state(const char* phase, int timeout_ms) {
  // Callers take the counter snapshot, then send the frame that elicits a
  // reply, then call this; the snapshot is passed through fresh_before_.
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::vector<uint64_t> after = frame_counts();
  auto all_advanced = [this, &after]() {
    for (size_t i = 0; i < after.size(); ++i) {
      if (after[i] <= fresh_before_[i]) return false;
    }
    return true;
  };
  while (!all_advanced() && std::chrono::steady_clock::now() < deadline) {
    openarm_->recv_all(1000);
    after = frame_counts();
  }
  std::vector<std::string> names(joint_names_.begin(),
                                 joint_names_.begin() + frame_counters_.size());
  const auto verdict = startup_safety::check_fresh_and_valid(
      fresh_before_, after, motor_positions(), position_bounds_, names,
      kMeasuredPositionMarginRad);
  if (!verdict.ok) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "%s: activation refused %s: %s", arm_prefix_.c_str(), phase,
                 verdict.reason.c_str());
  }
  return verdict.ok;
}

void OpenArmHW::latch_fault(const std::string& reason) {
  if (!fault_latched_) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"),
                 "%s: FAULT latched, commands blocked until the component is "
                 "deactivated and explicitly activated again: %s",
                 arm_prefix_.c_str(), reason.c_str());
  }
  fault_latched_ = true;
  fault_reason_ = reason;
}

std::vector<hardware_interface::StateInterface>
OpenArmHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], "temperature_rotor", &temp_rotor_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], "temperature_mos", &temp_mos_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmHW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Activating OpenArm V10...");
  active_ = false;
  fault_latched_ = false;
  fault_reason_.clear();

  const auto dwell = startup_safety::activation_dwell(
      configured_at_sec_, steady_now_sec(), min_inactive_sec_before_activate_);
  if (!dwell.ok) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmHW"), "%s: activation refused: %s",
                 arm_prefix_.c_str(), dwell.reason.c_str());
    return CallbackReturn::FAILURE;
  }

  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);

  if (verify_state_before_enable_) {
    // Same probe the vendor's motor-status check uses: a disable command,
    // which never energizes a motor, then wait for every motor's reply.
    fresh_before_ = frame_counts();
    openarm_->disable_all();
    if (!await_fresh_state("before enable", kFreshStateTimeoutMs)) {
      return CallbackReturn::FAILURE;
    }
  }

  fresh_before_ = frame_counts();
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();
  if (!await_fresh_state("after enable", kFreshStateTimeoutMs)) {
    openarm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
    return CallbackReturn::FAILURE;
  }

  if (auto_return_to_zero_) {
    // Upstream behavior, unchanged, including leaving the command buffers as
    // they were.
    RCLCPP_WARN(rclcpp::get_logger("OpenArmHW"),
                "auto_return_to_zero is enabled: moving to the zero pose now.");
    return_to_zero();
  } else {
    // Only a state received after enable reaches the command buffers, so the
    // first write() holds the measured pose instead of the 0.0 the buffers
    // were initialized with (or a previous activation's last command).
    const auto measured = motor_positions();
    for (size_t i = 0; i < measured.size() && i < pos_commands_.size(); ++i) {
      pos_commands_[i] = measured[i];
      vel_commands_[i] = 0.0;
      tau_commands_[i] = 0.0;
    }
  }

  stale_monitor_.reset(frame_counts(), steady_now_sec());
  active_ = true;
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmHW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Deactivating OpenArm V10...");
  active_ = false;

  // Disable all motors (like full_arm.cpp exit)
  for (int i = 0; i < 3; ++i) {
    openarm_->disable_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    openarm_->recv_all();
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmHW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->refresh_all();
  openarm_->recv_all();

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
    temp_rotor_states_[i] =
        static_cast<double>(arm_motors[i].get_state_trotor());
    temp_mos_states_[i] = static_cast<double>(arm_motors[i].get_state_tmos());
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      double motor_pos = gripper_motors[0].get_position();
      pos_states_[ARM_DOF] = motor_radians_to_joint(motor_pos);

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0].get_torque();

      temp_rotor_states_[ARM_DOF] =
          static_cast<double>(gripper_motors[0].get_state_trotor());
      temp_mos_states_[ARM_DOF] =
          static_cast<double>(gripper_motors[0].get_state_tmos());
    }
  }

  if (active_) {
    const double now = steady_now_sec();
    stale_monitor_.observe(frame_counts(), now);
    const int stale = stale_monitor_.first_stale(now, state_stale_timeout_sec_);
    if (stale >= 0) {
      latch_fault(joint_names_[static_cast<size_t>(stale)] +
                  ": no fresh state for more than " +
                  std::to_string(state_stale_timeout_sec_) + " s");
    }
    for (size_t i = 0; i < pos_states_.size(); ++i) {
      if (!std::isfinite(pos_states_[i])) {
        latch_fault(joint_names_[i] + ": non-finite measured position");
      }
    }
    if (fault_latched_) {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmHW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (!fault_latched_) {
    for (size_t i = 0; i < pos_commands_.size(); ++i) {
      if (!std::isfinite(pos_commands_[i]) || !std::isfinite(vel_commands_[i]) ||
          !std::isfinite(tau_commands_[i])) {
        latch_fault(joint_names_[i] + ": non-finite command");
        break;
      }
    }
  }
  if (fault_latched_) {
    // Nothing is sent: no new command reaches a motor while a fault stands.
    return hardware_interface::return_type::ERROR;
  }
  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back(
        {kp_[i], kd_[i], pos_commands_[i], vel_commands_[i], tau_commands_[i]});
  }
  openarm_->get_arm().mit_control_all(arm_params);
  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, motor_command, 0.0, 0.0}});
  }
  openarm_->recv_all(100);
  return hardware_interface::return_type::OK;
}

void OpenArmHW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero position...");

  openarm_->refresh_all();
  // Return arm to zero with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarm_->get_gripper().mit_control_all(
        {{gripper_kp_, gripper_kd_, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarm_->recv_all();
  const auto& arm_motors = openarm_->get_arm().get_motors();

  std::vector<double> start_pos(ARM_DOF, 0.0);
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    start_pos[i] = arm_motors[i].get_position();
  }

  const int steps = 200;
  const int step_ms = 10;

  for (int step = 0; step <= steps; ++step) {
    double t = static_cast<double>(step) / steps;  // 0.0 → 1.0

    std::vector<openarm::damiao_motor::MITParam> arm_params;
    for (size_t i = 0; i < ARM_DOF; ++i) {
      double target = start_pos[i] + t * (ZERO_POSITION[i] - start_pos[i]);
      arm_params.push_back({kp_[i], kd_[i], target, 0.0, 0.0});
    }
    openarm_->get_arm().mit_control_all(arm_params);

    if (hand_) {
      openarm_->get_gripper().mit_control_all(
          {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
    }

    openarm_->recv_all();
    std::this_thread::sleep_for(std::chrono::milliseconds(step_ms));
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Reached zero position");
}

// void OpenArmHW::return_to_zero() {
//   RCLCPP_INFO(rclcpp::get_logger("OpenArmHW"), "Returning to zero
//   position...");

//   // Return arm to zero with MIT control
//   std::vector<openarm::damiao_motor::MITParam> arm_params;
//   for (size_t i = 0; i < ARM_DOF; ++i) {
//     arm_params.push_back({kp_[i], kd_[i], 0.0, 0.0, 0.0});
//   }
//   openarm_->get_arm().mit_control_all(arm_params);

//   // Return gripper to zero if enabled
//   if (hand_) {
//     openarm_->get_gripper().mit_control_all(
//         {{GRIPPER_KP, GRIPPER_KD, GRIPPER_JOINT_0_POSITION, 0.0, 0.0}});
//   }
//   std::this_thread::sleep_for(std::chrono::microseconds(1000));
//   openarm_->recv_all();
// }

double OpenArmHW::joint_to_motor_radians(double joint_value) const {
  if (ee_type_ == "pinch_gripper") {
    // revolute: joint 0-1.5708 rad -> motor 0-1.5708
    return joint_value;
  } else {
    // parallel_link (prismatic): 0-0.044m -> 0 to -1.0472 rad
    return (joint_value / GRIPPER_JOINT_0_POSITION) * GRIPPER_MOTOR_1_RADIANS;
  }
}

double OpenArmHW::motor_radians_to_joint(double motor_radians) const {
  if (ee_type_ == "pinch_gripper") {
    // revolute:
    return motor_radians;
  } else {
    // parallel_link (prismatic)
    return GRIPPER_JOINT_0_POSITION * (motor_radians / GRIPPER_MOTOR_1_RADIANS);
  }
}

// // Gripper mapping helper functions
// double OpenArmHW::joint_to_motor_radians(double joint_value) const {
//   // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
//   return (joint_value / GRIPPER_JOINT_0_POSITION) *
//          GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
// }

// double OpenArmHW::motor_radians_to_joint(double motor_radians) const {
//   // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
//   return GRIPPER_JOINT_0_POSITION *
//          (motor_radians /
//           GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
// }

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmHW,
                       hardware_interface::SystemInterface)
