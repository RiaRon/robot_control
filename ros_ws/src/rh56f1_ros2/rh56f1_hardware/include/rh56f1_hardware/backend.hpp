// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "rh56f1_hardware/command_policy.hpp"
#include "rh56f1_hardware/transport.hpp"

namespace rh56f1_hardware {

// Latched until the next successful arm(). Recovery of the underlying
// condition never clears a fault by itself.
enum FaultBits : uint32_t {
  kFaultNone = 0,
  kFaultCommStale = 1u << 0,
  kFaultTransport = 1u << 1,
  kFaultDevice = 1u << 2,
  kFaultNonFiniteCommand = 1u << 3,
};

std::string describe_faults(uint32_t faults);

struct BackendConfig {
  PolicyConfig policy;
  double stale_timeout_sec = 0.0;
  double state_poll_period_sec = 0.0;
  // arm() needs this many successful reads since connect() ...
  int min_fresh_reads_to_arm = 0;
  // ... and at least this long between connect() and arm(): the plugin's
  // explicit-activation gate (see Rh56f1HW).
  double min_inactive_sec_before_arm = 0.0;
};

// Empty when valid, else what is wrong.
std::string validate(const BackendConfig& config);

struct BackendSnapshot {
  ActuatorStates states{};
  bool has_state = false;
  double last_good_read_sec = -std::numeric_limits<double>::infinity();
  uint64_t good_reads = 0;
  bool connected = false;
  bool armed = false;
  uint32_t faults = kFaultNone;
  std::string fault_detail;
  TransportError last_transport_error = TransportError::kNone;
  // Position clamping of the most recent decision, per actuator, and a
  // running count of decisions that clamped anything.
  std::array<bool, kActuatorCount> command_clamped{};
  ActuatorTargets clamp_requested{};
  ActuatorTargets clamp_result{};
  uint64_t clamp_events = 0;
  uint64_t writes = 0;
  ActuatorTargets last_sent{};
};

// One hand's transport plus everything that decides whether and what to send.
// All transport I/O happens in step() (the plugin's I/O worker thread, or a
// test calling it directly) and in the non-real-time connect()/disconnect();
// the ros2_control read()/write() side only uses the non-blocking try_*
// methods, so a slow serial link never stalls the controller_manager loop.
class Rh56f1Backend {
 public:
  Rh56f1Backend(std::unique_ptr<Rh56f1Transport> transport, BackendConfig config);

  // Non-real-time lifecycle. connect() opens the transport and restarts the
  // fresh-read count and the inactive timer; it never sends a command.
  bool connect(double now_sec, std::string* why);
  void disconnect();
  // Explicit arming: needs a connected transport, enough fresh reads, the
  // inactive dwell, and no device error in the latest state. On success the
  // step limiter and the pending command are seeded from the measured
  // position and every latched fault is cleared.
  bool arm(double now_sec, std::string* why);
  void disarm();

  // Real-time side; false only when the lock was busy (retry next cycle).
  // A non-finite command latches kFaultNonFiniteCommand and is not kept.
  bool try_submit_command(const ActuatorTargets& requested);
  bool try_snapshot(BackendSnapshot& out) const;

  // Worker side: polls state when due, detects staleness and device errors,
  // and (armed, no fault) sends at most one policy-approved command.
  void step(double now_sec);

  BackendSnapshot snapshot() const;
  const BackendConfig& config() const { return config_; }

 private:
  void latch(uint32_t fault, const std::string& detail);

  std::unique_ptr<Rh56f1Transport> transport_;
  BackendConfig config_;
  CommandPolicy policy_;

  // Serializes transport access between step() and connect()/disconnect().
  std::mutex io_mutex_;
  // Guards everything below; never held across transport I/O.
  mutable std::mutex data_mutex_;
  BackendSnapshot data_;
  double connected_at_sec_ = std::numeric_limits<double>::quiet_NaN();
  double last_poll_sec_ = -std::numeric_limits<double>::infinity();
  uint64_t reads_since_connect_ = 0;
  ActuatorTargets pending_{};
  bool has_pending_ = false;
};

}  // namespace rh56f1_hardware
