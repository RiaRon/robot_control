// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// STATUS: not implemented; Stage 5B-2 gate FAILED. See
// robot_control/docs/rh56f1-protocol-evidence.md for the blockers.
// connect() always fails, so selecting
// transport=rs485 refuses to run rather than silently doing something
// unverified — the mock transport and everything above this file (AssetSpec,
// CommandPolicy, the ros2_control plugin itself, xacro, controllers, launch)
// are unaffected and already validated.

#include "rh56f1_hardware/rs485_transport.hpp"

#include <utility>

namespace rh56f1_hardware {

Rs485Transport::Rs485Transport(std::string port, int baudrate, int device_id)
    : port_(std::move(port)), baudrate_(baudrate), device_id_(device_id) {}

bool Rs485Transport::connect() {
  // Deliberately refuses: per-hand IDs, raw<->URDF conversion, a single
  // official raw range, rates, timeout and power-on/comm-loss behavior are not
  // established (docs/rh56f1-protocol-evidence.md, gate 5B-2 items 7-14).
  // Do not open the port until they are.
  last_error_ = TransportError::kNotImplemented;
  connected_ = false;
  return false;
}

void Rs485Transport::disconnect() { connected_ = false; }

bool Rs485Transport::is_connected() const { return connected_; }

bool Rs485Transport::write_positions(const ActuatorTargets& /*targets_rad*/) {
  last_error_ = TransportError::kNotImplemented;
  return false;
}

bool Rs485Transport::read_state(ActuatorStates& /*out*/) {
  last_error_ = TransportError::kNotImplemented;
  return false;
}

TransportError Rs485Transport::last_error() const { return last_error_; }

std::string Rs485Transport::describe() const {
  return "rs485(not implemented):" + port_ + ":id=" + std::to_string(device_id_) +
         ":" + std::to_string(baudrate_) + "baud";
}

}  // namespace rh56f1_hardware
