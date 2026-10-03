// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/mock_transport.hpp"

#include <cmath>
#include <utility>

namespace rh56f1_hardware {

MockTransport::MockTransport(std::string identity) : identity_(std::move(identity)) {}

bool MockTransport::connect() {
  connect_calls_++;
  if (consume_injected_failure()) {
    return false;
  }
  connected_ = true;
  permanent_timeout_ = false;
  last_error_ = TransportError::kNone;
  return true;
}

void MockTransport::disconnect() {
  connected_ = false;
  permanent_timeout_ = false;
}

bool MockTransport::is_connected() const { return connected_; }

bool MockTransport::write_positions(const ActuatorTargets& targets_rad) {
  write_calls_++;
  if (!connected_) {
    last_error_ = TransportError::kNotConnected;
    return false;
  }
  for (double value : targets_rad) {
    if (!std::isfinite(value)) {
      last_error_ = TransportError::kInvalidCommand;
      return false;
    }
  }
  if (permanent_timeout_) {
    last_error_ = TransportError::kTimeout;
    return false;
  }
  if (consume_injected_failure()) {
    return false;
  }
  last_written_ = targets_rad;
  has_written_ = true;
  last_error_ = TransportError::kNone;
  return true;
}

bool MockTransport::read_state(ActuatorStates& out) {
  read_calls_++;
  if (!connected_) {
    last_error_ = TransportError::kNotConnected;
    return false;
  }
  if (permanent_timeout_) {
    last_error_ = TransportError::kTimeout;
    return false;
  }
  if (consume_injected_failure()) {
    return false;
  }
  if (fail_reads_after_ >= 0 && successful_reads_ >= fail_reads_after_) {
    last_error_ = TransportError::kTimeout;
    return false;
  }
  if (device_error_after_ >= 0 && successful_reads_ >= device_error_after_) {
    for (auto& actuator : state_) actuator.device_error = true;
  }
  if (has_written_) {
    for (std::size_t i = 0; i < kActuatorCount; ++i) {
      const double target = last_written_[i];
      const double current = state_[i].position_rad;
      state_[i].position_rad = current + (1.0 - lag_fraction_) * (target - current);
    }
  }
  out = state_;
  ++successful_reads_;
  last_error_ = TransportError::kNone;
  return true;
}

TransportError MockTransport::last_error() const { return last_error_; }

std::string MockTransport::describe() const { return "mock:" + identity_; }

void MockTransport::inject_failure(TransportError error, int count) {
  injected_error_ = error;
  injected_count_ = count;
}

void MockTransport::inject_permanent_timeout() { permanent_timeout_ = true; }

void MockTransport::set_lag_fraction(double fraction) { lag_fraction_ = fraction; }

void MockTransport::set_initial_positions(const ActuatorTargets& positions) {
  for (std::size_t i = 0; i < kActuatorCount; ++i) state_[i].position_rad = positions[i];
}

void MockTransport::fail_reads_after(int successful_reads) { fail_reads_after_ = successful_reads; }

void MockTransport::report_device_error_after(int successful_reads) {
  device_error_after_ = successful_reads;
}

void MockTransport::set_reported_current(std::size_t actuator_index, double amps) {
  state_.at(actuator_index).current_available = true;
  state_.at(actuator_index).current_a = amps;
}

void MockTransport::set_reported_temperature(std::size_t actuator_index, double celsius) {
  state_.at(actuator_index).temperature_available = true;
  state_.at(actuator_index).temperature_c = celsius;
}

void MockTransport::set_device_error(std::size_t actuator_index, bool has_error) {
  state_.at(actuator_index).device_error = has_error;
}

bool MockTransport::consume_injected_failure() {
  if (injected_count_ <= 0) {
    return false;
  }
  injected_count_--;
  last_error_ = injected_error_;
  return true;
}

}  // namespace rh56f1_hardware
