// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <string>

#include "rh56f1_hardware/transport.hpp"

namespace rh56f1_hardware {

// Placeholder for an RH56F1 RS-485 transport. It is a fail-closed stub: every
// operation refuses and nothing is ever written to a device.
//
// The installed hands are RS-485 (user confirmation, 2026-09-23). The layers a
// real implementation will reuse exist and are PTY-tested but are not linked
// into this plugin: rs485_codec.hpp (frames), serial_port.hpp (one owner per
// port, 8N1, timeouts) and register_client.hpp (validated request/reply,
// writes disabled by default).
// The real-transport gate (5B-2 in docs/rh56f1-protocol-evidence.md) still
// FAILS: per-hand bus IDs, the vendor-angle to URDF-joint conversion, a single
// official raw range, command/poll rate, reply timeout, and power-on /
// comm-loss behavior are not established. The vendor SDK in
// sim2real/vendor/inspire_ws is NOT used by this class.
//
// connect() must never command actuator motion (Rh56f1Transport contract).
class Rs485Transport : public Rh56f1Transport {
 public:
  Rs485Transport(std::string port, int baudrate, int device_id);

  bool connect() override;
  void disconnect() override;
  bool is_connected() const override;
  bool write_positions(const ActuatorTargets& targets_rad) override;
  bool read_state(ActuatorStates& out) override;
  TransportError last_error() const override;
  std::string describe() const override;

 private:
  std::string port_;
  int baudrate_;
  int device_id_;
  bool connected_ = false;
  TransportError last_error_ = TransportError::kNotImplemented;
};

}  // namespace rh56f1_hardware
