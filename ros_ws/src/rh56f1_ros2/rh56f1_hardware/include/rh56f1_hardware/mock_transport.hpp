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

// A transport that touches no real device, for unit tests and for the fake
// bringup's own tests of the Rh56f1HW plugin logic. State tracks the last
// accepted command instantaneously (no simulated dynamics) unless
// `set_lag_fraction` says otherwise. Every error path a real transport can
// take (disconnect, timeout, communication failure, invalid command) can be
// injected here so it is exercisable without a real device.
class MockTransport : public Rh56f1Transport {
 public:
  explicit MockTransport(std::string identity = "mock");

  bool connect() override;
  void disconnect() override;
  bool is_connected() const override;
  bool write_positions(const ActuatorTargets& targets_rad) override;
  bool read_state(ActuatorStates& out) override;
  TransportError last_error() const override;
  std::string describe() const override;

  // --- Test-only injection hooks below; a real transport has no equivalent. ---

  // The next `count` calls to connect()/write_positions()/read_state() each
  // fail once with `error`, then behavior reverts to normal. count=0 means
  // "inject nothing" (the default).
  void inject_failure(TransportError error, int count = 1);

  // Simulates a device that has stopped responding entirely: is_connected()
  // still returns true (the port is open), but every write/read fails with
  // kTimeout until reset() or reconnect (disconnect()+connect()).
  void inject_permanent_timeout();

  // 0.0 (default): read_state() reports the exact last commanded position
  // instantly. 1.0: read_state() never moves from whatever it last reported
  // (a stuck/jammed actuator). Values in between linearly interpolate one
  // step per read_state() call, for tests that want to see a transient
  // command/state mismatch rather than instant tracking.
  void set_lag_fraction(double fraction);

  // Runtime-probe knobs (the plugin's mock_* hardware parameters; ignored for
  // any other transport). Positions every actuator starts at; after that many
  // successful reads, reads start failing / a device error is reported.
  // A negative count disables that fault.
  void set_initial_positions(const ActuatorTargets& positions);
  void fail_reads_after(int successful_reads);
  void report_device_error_after(int successful_reads);

  void set_reported_current(std::size_t actuator_index, double amps);
  void set_reported_temperature(std::size_t actuator_index, double celsius);
  void set_device_error(std::size_t actuator_index, bool has_error);

  int connect_call_count() const { return connect_calls_; }
  int write_call_count() const { return write_calls_; }
  int read_call_count() const { return read_calls_; }
  const ActuatorTargets& last_written() const { return last_written_; }

 private:
  bool consume_injected_failure();

  std::string identity_;
  bool connected_ = false;
  bool permanent_timeout_ = false;
  double lag_fraction_ = 0.0;
  TransportError last_error_ = TransportError::kNone;
  TransportError injected_error_ = TransportError::kNone;
  int injected_count_ = 0;
  int connect_calls_ = 0;
  int write_calls_ = 0;
  int read_calls_ = 0;
  int fail_reads_after_ = -1;
  int device_error_after_ = -1;
  int successful_reads_ = 0;
  ActuatorStates state_{};
  ActuatorTargets last_written_{};
  bool has_written_ = false;
};

}  // namespace rh56f1_hardware
