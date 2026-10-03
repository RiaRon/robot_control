// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/rh56f1_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rh56f1_hardware/mock_transport.hpp"
#include "rh56f1_hardware/rs485_transport.hpp"

namespace rh56f1_hardware {

namespace {
rclcpp::Logger logger() { return rclcpp::get_logger("Rh56f1HW"); }

double steady_now_sec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

std::optional<std::string> param(const hardware_interface::HardwareInfo& info,
                                 const std::string& key) {
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return std::nullopt;
  return it->second;
}

std::optional<double> to_double(const std::string& text) {
  try {
    size_t used = 0;
    const double value = std::stod(text, &used);
    if (used != text.size() || !std::isfinite(value)) return std::nullopt;
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<int> to_int(const std::string& text) {
  try {
    size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size()) return std::nullopt;
    return value;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

// Required numeric hardware parameter; logs and returns nullopt if absent or
// malformed. There are deliberately no silent defaults for safety values.
std::optional<double> required_double(const hardware_interface::HardwareInfo& info,
                                      const std::string& key) {
  const auto text = param(info, key);
  if (!text) {
    RCLCPP_ERROR(logger(), "required hardware parameter '%s' is missing", key.c_str());
    return std::nullopt;
  }
  const auto value = to_double(*text);
  if (!value) {
    RCLCPP_ERROR(logger(), "hardware parameter '%s'='%s' is not a finite number",
                 key.c_str(), text->c_str());
  }
  return value;
}
}  // namespace

Rh56f1HW::~Rh56f1HW() { stop_worker(); }

bool Rh56f1HW::parse_config(const hardware_interface::HardwareInfo& info,
                            BackendConfig& config) {
  side_ = param(info, "side").value_or("");
  if (side_ != "left" && side_ != "right") {
    RCLCPP_ERROR(logger(), "hardware parameter 'side' must be 'left' or 'right', got '%s'",
                 side_.c_str());
    return false;
  }
  transport_kind_ = param(info, "transport").value_or("");
  if (transport_kind_ != "mock" && transport_kind_ != "rs485") {
    RCLCPP_ERROR(logger(),
                 "%s hand: hardware parameter 'transport' must be 'mock' or 'rs485' "
                 "(no default), got '%s'",
                 side_.c_str(), transport_kind_.c_str());
    return false;
  }

  // Bind every canonical actuator to exactly one declared joint, by name.
  if (info.joints.size() != kActuatorCount) {
    RCLCPP_ERROR(logger(), "%s hand: expected %zu joints, got %zu", side_.c_str(),
                 kActuatorCount, info.joints.size());
    return false;
  }
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const std::string key = std::string("actuator_") + kActuatorKeys[i];
    const auto name = param(info, key);
    if (!name || name->empty()) {
      RCLCPP_ERROR(logger(), "%s hand: required hardware parameter '%s' is missing",
                   side_.c_str(), key.c_str());
      return false;
    }
    const auto matches = std::count_if(info.joints.begin(), info.joints.end(),
                                       [&](const auto& joint) { return joint.name == *name; });
    if (matches != 1) {
      RCLCPP_ERROR(logger(), "%s hand: %s='%s' matches %ld declared joints, expected 1",
                   side_.c_str(), key.c_str(), name->c_str(), static_cast<long>(matches));
      return false;
    }
    if (std::find(joint_names_.begin(), joint_names_.begin() + i, *name) !=
        joint_names_.begin() + i) {
      RCLCPP_ERROR(logger(), "%s hand: joint '%s' is bound to two actuators", side_.c_str(),
                   name->c_str());
      return false;
    }
    joint_names_[i] = *name;

    // Position limits come from the joint's own position command interface
    // (rendered from the canonical URDF), never from a separate table here.
    const auto& joint = *std::find_if(info.joints.begin(), info.joints.end(),
                                      [&](const auto& j) { return j.name == *name; });
    const auto command = std::find_if(joint.command_interfaces.begin(),
                                      joint.command_interfaces.end(), [](const auto& c) {
                                        return c.name == hardware_interface::HW_IF_POSITION;
                                      });
    if (command == joint.command_interfaces.end()) {
      RCLCPP_ERROR(logger(), "%s hand: joint '%s' declares no position command interface",
                   side_.c_str(), name->c_str());
      return false;
    }
    const auto lower = to_double(command->min);
    const auto upper = to_double(command->max);
    if (!lower || !upper) {
      RCLCPP_ERROR(logger(),
                   "%s hand: joint '%s' position command interface needs numeric min/max "
                   "(got '%s'/'%s')",
                   side_.c_str(), name->c_str(), command->min.c_str(), command->max.c_str());
      return false;
    }
    config.policy.limits[i] = {*lower, *upper};
  }

  const auto velocity = required_double(info, "max_velocity_rad_s");
  const auto step = required_double(info, "max_step_rad");
  const auto write_period = required_double(info, "min_write_period_sec");
  const auto stale = required_double(info, "stale_timeout_sec");
  const auto poll = required_double(info, "state_poll_period_sec");
  const auto dwell = required_double(info, "min_inactive_sec_before_activate");
  const auto fresh_reads = param(info, "min_fresh_reads_to_activate");
  const auto fresh_reads_value = fresh_reads ? to_int(*fresh_reads) : std::nullopt;
  if (!fresh_reads_value) {
    RCLCPP_ERROR(logger(), "%s hand: min_fresh_reads_to_activate must be an integer",
                 side_.c_str());
  }
  if (!velocity || !step || !write_period || !stale || !poll || !dwell || !fresh_reads_value) {
    return false;
  }
  config.policy.max_velocity_rad_s = *velocity;
  config.policy.max_step_rad = *step;
  config.policy.min_write_period_sec = *write_period;
  config.stale_timeout_sec = *stale;
  config.state_poll_period_sec = *poll;
  config.min_inactive_sec_before_arm = *dwell;
  config.min_fresh_reads_to_arm = *fresh_reads_value;
  const std::string problem = validate(config);
  if (!problem.empty()) {
    RCLCPP_ERROR(logger(), "%s hand: %s", side_.c_str(), problem.c_str());
    return false;
  }
  worker_period_sec_ =
      std::max(0.001, std::min(config.state_poll_period_sec,
                               std::max(config.policy.min_write_period_sec, 0.002)) / 2.0);

  port_ = param(info, "port").value_or("");
  baudrate_ = to_int(param(info, "baudrate").value_or("0")).value_or(0);
  device_id_ = to_int(param(info, "device_id").value_or("0")).value_or(0);
  return true;
}

std::unique_ptr<Rh56f1Transport> Rh56f1HW::make_transport(
    const hardware_interface::HardwareInfo& info, bool& ok) const {
  ok = true;
  const bool has_mock_params = param(info, "mock_initial_positions") ||
                               param(info, "mock_fail_reads_after") ||
                               param(info, "mock_device_error_after_reads");
  if (transport_kind_ == "rs485") {
    if (has_mock_params) {
      RCLCPP_ERROR(logger(), "%s hand: mock_* parameters are only valid with transport=mock",
                   side_.c_str());
      ok = false;
    }
    return std::make_unique<Rs485Transport>(port_, baudrate_, device_id_);
  }
  auto mock = std::make_unique<MockTransport>(side_);
  if (const auto text = param(info, "mock_initial_positions")) {
    ActuatorTargets positions{};
    std::stringstream stream(*text);
    std::string item;
    std::size_t count = 0;
    while (std::getline(stream, item, ',')) {
      const auto value = to_double(item);
      if (!value || count >= kActuatorCount) {
        ok = false;
        break;
      }
      positions[count++] = *value;
    }
    if (!ok || count != kActuatorCount) {
      RCLCPP_ERROR(logger(), "%s hand: mock_initial_positions needs 6 numbers", side_.c_str());
      ok = false;
    } else {
      mock->set_initial_positions(positions);
    }
  }
  if (const auto text = param(info, "mock_fail_reads_after")) {
    const auto value = to_int(*text);
    if (!value) ok = false; else mock->fail_reads_after(*value);
  }
  if (const auto text = param(info, "mock_device_error_after_reads")) {
    const auto value = to_int(*text);
    if (!value) ok = false; else mock->report_device_error_after(*value);
  }
  return mock;
}

hardware_interface::CallbackReturn Rh56f1HW::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  BackendConfig config;
  if (!parse_config(info, config)) {
    return CallbackReturn::ERROR;
  }
  bool ok = false;
  auto transport = make_transport(info, ok);
  if (!ok) {
    return CallbackReturn::ERROR;
  }
  const std::string identity = transport->describe();
  backend_ = std::make_unique<Rh56f1Backend>(std::move(transport), config);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  pos_commands_.fill(nan);
  pos_states_.fill(nan);
  vel_states_.fill(nan);
  effort_states_.fill(nan);
  current_states_.fill(nan);
  temperature_states_.fill(nan);
  comm_ok_states_.fill(0.0);
  fault_states_.fill(0.0);
  clamped_states_.fill(0.0);
  is_mock_states_.fill(transport_kind_ == "mock" ? 1.0 : 0.0);
  writes_states_.fill(0.0);

  RCLCPP_INFO(logger(), "%s hand initialized: %s", side_.c_str(), identity.c_str());
  if (transport_kind_ == "mock") {
    RCLCPP_WARN(logger(), "%s hand uses the MOCK transport: its state is simulated, not measured",
                side_.c_str());
  }
  return CallbackReturn::SUCCESS;
}

void Rh56f1HW::start_worker() {
  stop_worker();
  worker_running_ = true;
  worker_ = std::thread([this]() {
    const auto period = std::chrono::duration<double>(worker_period_sec_);
    while (worker_running_) {
      backend_->step(steady_now_sec());
      std::this_thread::sleep_for(period);
    }
  });
}

void Rh56f1HW::stop_worker() {
  worker_running_ = false;
  if (worker_.joinable()) worker_.join();
}

hardware_interface::CallbackReturn Rh56f1HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  armed_ = false;
  std::string why;
  if (!backend_->connect(steady_now_sec(), &why)) {
    RCLCPP_ERROR(logger(), "%s hand: %s", side_.c_str(), why.c_str());
    return CallbackReturn::ERROR;
  }
  start_worker();
  RCLCPP_INFO(logger(), "%s hand configured: reading state only, commands disabled",
              side_.c_str());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rh56f1HW::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  armed_ = false;
  stop_worker();
  backend_->disconnect();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rh56f1HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  armed_ = false;
  std::string why;
  if (!backend_->arm(steady_now_sec(), &why)) {
    RCLCPP_ERROR(logger(), "%s hand: activation refused: %s", side_.c_str(), why.c_str());
    return CallbackReturn::FAILURE;
  }
  const auto snapshot = backend_->snapshot();
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    pos_commands_[i] = snapshot.last_sent[i];
  }
  last_snapshot_ = snapshot;
  reported_faults_ = kFaultNone;
  armed_ = true;
  RCLCPP_WARN(logger(), "%s hand ARMED: commands enabled, holding the measured position",
              side_.c_str());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rh56f1HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  armed_ = false;
  backend_->disarm();
  RCLCPP_INFO(logger(), "%s hand disarmed: commands disabled", side_.c_str());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rh56f1HW::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  armed_ = false;
  backend_->disarm();
  stop_worker();
  backend_->disconnect();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rh56f1HW::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  armed_ = false;
  backend_->disarm();
  stop_worker();
  backend_->disconnect();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Rh56f1HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const auto& name = joint_names_[i];
    interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &pos_states_[i]);
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &vel_states_[i]);
    // No torque measurement is confirmed for this hand; NaN says so.
    interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &effort_states_[i]);
    interfaces.emplace_back(name, "current", &current_states_[i]);
    interfaces.emplace_back(name, "temperature", &temperature_states_[i]);
    interfaces.emplace_back(name, "comm_ok", &comm_ok_states_[i]);
    interfaces.emplace_back(name, "fault", &fault_states_[i]);
    interfaces.emplace_back(name, "command_clamped", &clamped_states_[i]);
    interfaces.emplace_back(name, "is_mock", &is_mock_states_[i]);
    interfaces.emplace_back(name, "writes", &writes_states_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> Rh56f1HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION,
                            &pos_commands_[i]);
  }
  return interfaces;
}

void Rh56f1HW::report_clamps(const BackendSnapshot& snapshot) {
  if (snapshot.clamp_events == reported_clamp_events_) return;
  std::string detail;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (!snapshot.command_clamped[i]) continue;
    detail += (detail.empty() ? "" : ", ") + joint_names_[i] + " " +
              std::to_string(snapshot.clamp_requested[i]) + " -> " +
              std::to_string(snapshot.clamp_result[i]);
  }
  RCLCPP_WARN_THROTTLE(logger(), throttle_clock_, 1000,
                       "%s hand: position command clamped to joint limits (%lu clamped "
                       "commands so far): %s",
                       side_.c_str(), static_cast<unsigned long>(snapshot.clamp_events),
                       detail.c_str());
  reported_clamp_events_ = snapshot.clamp_events;
}

hardware_interface::return_type Rh56f1HW::read(const rclcpp::Time& /*time*/,
                                               const rclcpp::Duration& /*period*/) {
  BackendSnapshot snapshot;
  if (backend_->try_snapshot(snapshot)) {
    last_snapshot_ = snapshot;
  }
  const auto& s = last_snapshot_;
  const double now = steady_now_sec();
  const bool fresh = s.connected && now - s.last_good_read_sec <= backend_->config().stale_timeout_sec;
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (s.has_state) {
      pos_states_[i] = s.states[i].position_rad;
      vel_states_[i] = s.states[i].velocity_available ? s.states[i].velocity_rad_s
                                                      : std::numeric_limits<double>::quiet_NaN();
      current_states_[i] = s.states[i].current_available ? s.states[i].current_a
                                                         : std::numeric_limits<double>::quiet_NaN();
      temperature_states_[i] = s.states[i].temperature_available
                                   ? s.states[i].temperature_c
                                   : std::numeric_limits<double>::quiet_NaN();
    }
    comm_ok_states_[i] = fresh ? 1.0 : 0.0;
    fault_states_[i] = static_cast<double>(s.faults);
    clamped_states_[i] = s.command_clamped[i] ? 1.0 : 0.0;
    writes_states_[i] = static_cast<double>(s.writes);
  }
  report_clamps(s);

  if (armed_ && s.faults != kFaultNone) {
    if (s.faults != reported_faults_) {
      RCLCPP_ERROR(logger(),
                   "%s hand FAULT latched (%s): commands blocked until deactivate + explicit "
                   "activate. %s",
                   side_.c_str(), describe_faults(s.faults).c_str(), s.fault_detail.c_str());
      reported_faults_ = s.faults;
    }
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Rh56f1HW::write(const rclcpp::Time& /*time*/,
                                                const rclcpp::Duration& /*period*/) {
  if (!armed_) {
    return hardware_interface::return_type::OK;
  }
  backend_->try_submit_command(pos_commands_);
  if (last_snapshot_.faults != kFaultNone) {
    return hardware_interface::return_type::ERROR;
  }
  const bool finite = std::all_of(pos_commands_.begin(), pos_commands_.end(),
                                  [](double v) { return std::isfinite(v); });
  return finite ? hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

}  // namespace rh56f1_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rh56f1_hardware::Rh56f1HW, hardware_interface::SystemInterface)
