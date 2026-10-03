// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/backend.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rh56f1_hardware {

std::string describe_faults(uint32_t faults) {
  if (faults == kFaultNone) return "none";
  std::string out;
  auto add = [&out](const char* name) { out += (out.empty() ? "" : "|") + std::string(name); };
  if (faults & kFaultCommStale) add("comm_stale");
  if (faults & kFaultTransport) add("transport_error");
  if (faults & kFaultDevice) add("device_error");
  if (faults & kFaultNonFiniteCommand) add("non_finite_command");
  return out;
}

std::string validate(const BackendConfig& config) {
  const std::string policy = validate(config.policy);
  if (!policy.empty()) return policy;
  if (!std::isfinite(config.stale_timeout_sec) || config.stale_timeout_sec <= 0.0) {
    return "stale_timeout_sec must be a positive number";
  }
  if (!std::isfinite(config.state_poll_period_sec) || config.state_poll_period_sec <= 0.0) {
    return "state_poll_period_sec must be a positive number";
  }
  if (config.state_poll_period_sec >= config.stale_timeout_sec) {
    return "state_poll_period_sec must be shorter than stale_timeout_sec";
  }
  if (config.min_fresh_reads_to_arm < 1) {
    return "min_fresh_reads_to_arm must be at least 1";
  }
  if (!std::isfinite(config.min_inactive_sec_before_arm) ||
      config.min_inactive_sec_before_arm < 0.0) {
    return "min_inactive_sec_before_arm must be a non-negative number";
  }
  return "";
}

Rh56f1Backend::Rh56f1Backend(std::unique_ptr<Rh56f1Transport> transport,
                             BackendConfig config)
    : transport_(std::move(transport)), config_(config), policy_(config.policy) {}

bool Rh56f1Backend::connect(double now_sec, std::string* why) {
  bool ok = false;
  TransportError error = TransportError::kNone;
  {
    std::lock_guard<std::mutex> io(io_mutex_);
    ok = transport_->is_connected() || transport_->connect();
    error = transport_->last_error();
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_.armed = false;
  data_.connected = ok;
  data_.last_transport_error = error;
  policy_.unseed();
  has_pending_ = false;
  reads_since_connect_ = 0;
  last_poll_sec_ = -std::numeric_limits<double>::infinity();
  connected_at_sec_ = ok ? now_sec : std::numeric_limits<double>::quiet_NaN();
  if (!ok && why) *why = std::string("connect() failed: ") + to_string(error);
  return ok;
}

void Rh56f1Backend::disconnect() {
  {
    std::lock_guard<std::mutex> io(io_mutex_);
    transport_->disconnect();
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_.armed = false;
  data_.connected = false;
  policy_.unseed();
  has_pending_ = false;
}

bool Rh56f1Backend::arm(double now_sec, std::string* why) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  auto refuse = [why](const std::string& reason) {
    if (why) *why = reason;
    return false;
  };
  if (!data_.connected) return refuse("transport is not connected");
  if (!(now_sec - connected_at_sec_ >= config_.min_inactive_sec_before_arm)) {
    return refuse("activation requested " + std::to_string(now_sec - connected_at_sec_) +
                  " s after configure; explicit activation requires at least " +
                  std::to_string(config_.min_inactive_sec_before_arm) +
                  " s inactive (auto-activation at startup is refused)");
  }
  if (reads_since_connect_ < static_cast<uint64_t>(config_.min_fresh_reads_to_arm)) {
    return refuse("only " + std::to_string(reads_since_connect_) +
                  " successful state reads since configure; need " +
                  std::to_string(config_.min_fresh_reads_to_arm));
  }
  if (now_sec - data_.last_good_read_sec > config_.stale_timeout_sec) {
    return refuse("no fresh state within stale_timeout_sec");
  }
  ActuatorTargets current{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    if (data_.states[i].device_error) {
      return refuse("actuator " + std::to_string(i) + " reports a device error");
    }
    if (!std::isfinite(data_.states[i].position_rad)) {
      return refuse("actuator " + std::to_string(i) + " reports a non-finite position");
    }
    current[i] = data_.states[i].position_rad;
  }
  policy_.reset(current, now_sec);
  pending_ = current;
  has_pending_ = true;
  data_.faults = kFaultNone;
  data_.fault_detail.clear();
  data_.command_clamped.fill(false);
  data_.last_sent = current;
  data_.armed = true;
  return true;
}

void Rh56f1Backend::disarm() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_.armed = false;
  policy_.unseed();
  has_pending_ = false;
}

bool Rh56f1Backend::try_submit_command(const ActuatorTargets& requested) {
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  const bool finite = std::all_of(requested.begin(), requested.end(),
                                  [](double v) { return std::isfinite(v); });
  if (!finite) {
    latch(kFaultNonFiniteCommand, "a non-finite position command was written");
    return true;
  }
  pending_ = requested;
  has_pending_ = true;
  return true;
}

bool Rh56f1Backend::try_snapshot(BackendSnapshot& out) const {
  std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return false;
  out = data_;
  return true;
}

BackendSnapshot Rh56f1Backend::snapshot() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return data_;
}

void Rh56f1Backend::latch(uint32_t fault, const std::string& detail) {
  if ((data_.faults & fault) == 0) {
    data_.fault_detail += (data_.fault_detail.empty() ? "" : "; ") + detail;
  }
  data_.faults |= fault;
}

void Rh56f1Backend::step(double now_sec) {
  bool connected = false;
  bool poll_due = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    connected = data_.connected;
    poll_due = now_sec - last_poll_sec_ >= config_.state_poll_period_sec;
    if (poll_due) last_poll_sec_ = now_sec;
  }
  if (!connected) return;

  if (poll_due) {
    ActuatorStates states{};
    bool ok = false;
    TransportError error = TransportError::kNone;
    {
      std::lock_guard<std::mutex> io(io_mutex_);
      ok = transport_->read_state(states);
      error = transport_->last_error();
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    data_.last_transport_error = error;
    if (ok) {
      data_.states = states;
      data_.has_state = true;
      data_.last_good_read_sec = now_sec;
      ++data_.good_reads;
      ++reads_since_connect_;
      for (std::size_t i = 0; i < kActuatorCount; ++i) {
        if (states[i].device_error) {
          latch(kFaultDevice, "actuator " + std::to_string(i) + " reported a device error");
        }
      }
    }
  }

  ActuatorTargets targets{};
  bool send = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (now_sec - data_.last_good_read_sec > config_.stale_timeout_sec) {
      latch(kFaultCommStale, "no successful state read within stale_timeout_sec");
    }
    if (!data_.armed || data_.faults != kFaultNone || !has_pending_) return;
    const auto decision = policy_.decide(pending_, now_sec);
    if (decision.verdict == CommandPolicy::Verdict::kRejectedNonFinite) {
      latch(kFaultNonFiniteCommand, "a non-finite position command reached the policy");
      return;
    }
    if (decision.verdict != CommandPolicy::Verdict::kSend) return;
    data_.command_clamped = decision.position_clamped;
    const bool clamped = std::any_of(decision.position_clamped.begin(),
                                     decision.position_clamped.end(),
                                     [](bool v) { return v; });
    if (clamped) {
      ++data_.clamp_events;
      data_.clamp_requested = decision.requested;
      data_.clamp_result = decision.clamped_to;
    }
    targets = decision.targets;
    send = true;
  }
  if (!send) return;

  bool ok = false;
  TransportError error = TransportError::kNone;
  {
    std::lock_guard<std::mutex> io(io_mutex_);
    ok = transport_->write_positions(targets);
    error = transport_->last_error();
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_.last_transport_error = error;
  if (ok) {
    ++data_.writes;
    data_.last_sent = targets;
  } else {
    latch(kFaultTransport, std::string("write_positions() failed: ") + to_string(error));
  }
}

}  // namespace rh56f1_hardware
