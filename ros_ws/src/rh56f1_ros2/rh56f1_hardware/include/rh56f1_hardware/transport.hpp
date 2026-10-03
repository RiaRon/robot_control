// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <array>
#include <cstddef>
#include <string>

namespace rh56f1_hardware {

// One hand: 6 independent actuators (thumb1, thumb2, index1, middle1, ring1,
// pinky1), in that fixed order. Passive/mimic joints (thumb3/4, index2,
// middle2, ring2, pinky2) are not actuators and are never part of this
// transport's contract.
//
// This is the plugin's own array order, not the device's. The official
// RH56F1 register order (manual tables 35/40) is little, ring, middle, index,
// thumb bend, thumb rotation; a real transport must remap by name, and which
// of thumb1/thumb2 is the vendor's bend vs rotation actuator is not yet
// confirmed (docs/rh56f1-protocol-evidence.md).
inline constexpr std::size_t kActuatorCount = 6;

enum class TransportError {
  kNone,
  kNotConnected,
  kTimeout,
  kCommunicationError,
  kInvalidResponse,
  // A command this transport was asked to send was rejected before anything
  // was written to the device (NaN/Inf, or otherwise not a sendable command).
  kInvalidCommand,
  kNotImplemented,
};

const char* to_string(TransportError error);

// One actuator's measured state, in canonical radians. `position_rad` is
// always populated on a successful read; the rest are optional because not
// every transport/protocol necessarily exposes them (see
// Rh56f1Transport::read_state doc below and the per-transport implementation
// notes on which of these are backed by an official protocol field versus
// left unavailable).
struct ActuatorState {
  double position_rad = 0.0;
  bool velocity_available = false;
  double velocity_rad_s = 0.0;
  bool current_available = false;
  double current_a = 0.0;
  bool temperature_available = false;
  double temperature_c = 0.0;
  // True if the device itself reported a fault/error code for this actuator
  // (over-current, over-temperature, stall, ...), independent of whether this
  // transport call itself succeeded.
  bool device_error = false;
};

using ActuatorStates = std::array<ActuatorState, kActuatorCount>;
using ActuatorTargets = std::array<double, kActuatorCount>;

// Backend abstraction for one RH56F1 hand (one side: left or right). A
// concrete implementation talks to either a real device (serial/CAN) or, for
// MockTransport, nothing at all — every normal and error path must be
// exercisable through this interface without a real device attached.
//
// Contract every implementation must uphold:
//  - connect() opens the device/port and may read its status, but must never
//    command any actuator motion by itself.
//  - write_positions() is the only method that may cause physical motion, and
//    only after connect() succeeded; a caller (Rh56f1HW) is additionally
//    responsible for not calling it before its own explicit enable/activation
//    step, per OPENARM_RH56F1_INTEGRATION_STATUS.md Stage 4.
//  - No method may throw for a normal communication failure: report it via
//    the boolean return value and last_error(), so the ros2_control read()/
//    write() cycle can keep running and surface a fault state instead of
//    unwinding the control loop.
class Rh56f1Transport {
 public:
  virtual ~Rh56f1Transport() = default;

  // Opens the underlying device/port. Must not send any actuator command.
  // Returns false (see last_error()) if the device could not be opened.
  virtual bool connect() = 0;

  // Closes the underlying device/port. Safe to call when not connected.
  virtual void disconnect() = 0;

  virtual bool is_connected() const = 0;

  // Commands all 6 actuators to the given canonical-radian targets in one
  // call (a single hand's actuators are one device on one port; there is no
  // partial-hand write). Returns false on any communication failure and
  // leaves the device's last accepted command unspecified from this
  // transport's point of view — the caller (Rh56f1HW) is responsible for
  // deciding what "hold" means when writes start failing.
  virtual bool write_positions(const ActuatorTargets& targets_rad) = 0;

  // Reads all 6 actuators' current state. Returns false on any communication
  // failure; `out` is left unspecified in that case; the caller must not
  // trust `out` unless this returns true.
  virtual bool read_state(ActuatorStates& out) = 0;

  virtual TransportError last_error() const = 0;

  // Human-readable identity for logging (e.g. "mock:right", "rs485:/dev/ttyUSB0:id=2").
  virtual std::string describe() const = 0;
};

}  // namespace rh56f1_hardware
