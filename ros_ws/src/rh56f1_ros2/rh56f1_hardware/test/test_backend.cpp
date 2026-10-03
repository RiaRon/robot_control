// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <limits>
#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "rh56f1_hardware/backend.hpp"
#include "rh56f1_hardware/mock_transport.hpp"

namespace rh56f1_hardware {
namespace {

ActuatorTargets Filled(double value) {
  ActuatorTargets t;
  t.fill(value);
  return t;
}

BackendConfig Config() {
  BackendConfig config;
  config.policy.limits.fill(JointLimits{0.0, 1.5});
  config.policy.max_velocity_rad_s = 2.0;
  config.policy.max_step_rad = 0.02;
  config.policy.min_write_period_sec = 0.1;
  config.stale_timeout_sec = 0.25;
  config.state_poll_period_sec = 0.02;
  config.min_fresh_reads_to_arm = 3;
  config.min_inactive_sec_before_arm = 2.0;
  return config;
}

struct Fixture {
  MockTransport* mock = nullptr;
  std::unique_ptr<Rh56f1Backend> backend;
  explicit Fixture(double initial = 0.4) {
    auto transport = std::make_unique<MockTransport>("test");
    transport->set_initial_positions(Filled(initial));
    mock = transport.get();
    backend = std::make_unique<Rh56f1Backend>(std::move(transport), Config());
  }
  // Steps every 20 ms from `from` to `to` inclusive.
  void run(double from, double to) {
    for (double t = from; t <= to + 1e-9; t += 0.02) backend->step(t);
  }
  void connect_and_arm() {
    std::string why;
    ASSERT_TRUE(backend->connect(0.0, &why)) << why;
    run(0.0, 2.1);
    ASSERT_TRUE(backend->arm(2.1, &why)) << why;
  }
};

TEST(Backend, ConfigValidation) {
  EXPECT_EQ(validate(Config()), "");
  auto bad = Config();
  bad.state_poll_period_sec = 0.3;
  EXPECT_NE(validate(bad), "");
  bad = Config();
  bad.min_fresh_reads_to_arm = 0;
  EXPECT_NE(validate(bad), "");
}

TEST(Backend, ConfigureReadsButNeverWrites) {
  Fixture f;
  std::string why;
  ASSERT_TRUE(f.backend->connect(0.0, &why));
  f.run(0.0, 5.0);
  EXPECT_GT(f.mock->read_call_count(), 0);
  EXPECT_EQ(f.mock->write_call_count(), 0);
  EXPECT_FALSE(f.backend->snapshot().armed);
}

TEST(Backend, ImmediateAutoActivationIsRefused) {
  Fixture f;
  std::string why;
  ASSERT_TRUE(f.backend->connect(0.0, &why));
  f.run(0.0, 0.1);
  EXPECT_FALSE(f.backend->arm(0.1, &why));
  EXPECT_THAT(why, ::testing::HasSubstr("auto-activation"));
  f.run(0.12, 1.0);
  EXPECT_EQ(f.mock->write_call_count(), 0);
}

TEST(Backend, ArmingSeedsFromMeasuredPositionSoTheFirstWriteHolds) {
  Fixture f(0.4);
  f.connect_and_arm();
  f.run(2.12, 2.3);
  ASSERT_GT(f.mock->write_call_count(), 0);
  EXPECT_THAT(f.mock->last_written(), ::testing::Each(::testing::DoubleEq(0.4)));
}

TEST(Backend, CommandsMoveAtMostOneStepPerWrite) {
  Fixture f(0.4);
  f.connect_and_arm();
  ASSERT_TRUE(f.backend->try_submit_command(Filled(1.0)));
  f.run(2.12, 2.25);
  for (double v : f.mock->last_written()) {
    EXPECT_GT(v, 0.4);
    EXPECT_LE(v, 0.4 + 2 * 0.02 + 1e-9);
  }
}

TEST(Backend, OutOfLimitCommandIsClampedAndCounted) {
  Fixture f(1.49);
  f.connect_and_arm();
  ActuatorTargets command = Filled(1.49);
  command[2] = 5.0;
  ASSERT_TRUE(f.backend->try_submit_command(command));
  f.run(2.12, 2.4);
  const auto s = f.backend->snapshot();
  EXPECT_GT(s.clamp_events, 0u);
  EXPECT_TRUE(s.command_clamped[2]);
  EXPECT_DOUBLE_EQ(s.clamp_requested[2], 5.0);
  EXPECT_DOUBLE_EQ(s.clamp_result[2], 1.5);
  EXPECT_LE(f.mock->last_written()[2], 1.5);
  EXPECT_EQ(s.faults, kFaultNone);
}

TEST(Backend, NonFiniteCommandLatchesAndStopsWrites) {
  Fixture f;
  f.connect_and_arm();
  f.run(2.12, 2.3);
  ActuatorTargets command = Filled(0.4);
  command[0] = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(f.backend->try_submit_command(command));
  const int writes = f.mock->write_call_count();
  ASSERT_TRUE(f.backend->try_submit_command(Filled(0.4)));
  f.run(2.32, 3.0);
  EXPECT_EQ(f.mock->write_call_count(), writes);
  EXPECT_TRUE(f.backend->snapshot().faults & kFaultNonFiniteCommand);
}

TEST(Backend, StaleStateLatchesAndRecoveryDoesNotResumeOldCommand) {
  Fixture f;
  f.connect_and_arm();
  f.run(2.12, 2.3);
  f.mock->inject_failure(TransportError::kTimeout, 20);  // ~0.4 s of failed reads
  f.run(2.32, 2.8);
  EXPECT_TRUE(f.backend->snapshot().faults & kFaultCommStale);
  const int writes = f.mock->write_call_count();
  ASSERT_TRUE(f.backend->try_submit_command(Filled(0.9)));
  f.run(2.82, 4.0);  // reads succeed again
  EXPECT_EQ(f.mock->write_call_count(), writes);
  EXPECT_TRUE(f.backend->snapshot().faults & kFaultCommStale);

  // Re-arming reseeds from the measured position, not the stale 0.9 command.
  f.backend->disarm();
  std::string why;
  ASSERT_TRUE(f.backend->arm(4.0, &why)) << why;
  EXPECT_EQ(f.backend->snapshot().faults, kFaultNone);
  f.run(4.02, 4.2);
  EXPECT_THAT(f.mock->last_written(), ::testing::Each(::testing::DoubleEq(0.4)));
}

TEST(Backend, DeviceErrorLatchesAndBlocksArming) {
  Fixture f;
  f.connect_and_arm();
  f.mock->set_device_error(1, true);
  f.run(2.12, 2.4);
  EXPECT_TRUE(f.backend->snapshot().faults & kFaultDevice);
  f.backend->disarm();
  std::string why;
  EXPECT_FALSE(f.backend->arm(2.5, &why));
  EXPECT_THAT(why, ::testing::HasSubstr("device error"));
}

TEST(Backend, WriteFailureLatchesTransportFault) {
  Fixture f;
  f.connect_and_arm();
  f.mock->inject_permanent_timeout();
  f.run(2.12, 2.3);
  EXPECT_TRUE(f.backend->snapshot().faults & (kFaultTransport | kFaultCommStale));
}

TEST(Backend, DisarmStopsWritesAndStepNeverReconnects) {
  Fixture f;
  f.connect_and_arm();
  f.backend->disarm();
  const int writes = f.mock->write_call_count();
  const int connects = f.mock->connect_call_count();
  f.mock->disconnect();
  f.run(2.12, 3.0);
  EXPECT_EQ(f.mock->write_call_count(), writes);
  EXPECT_EQ(f.mock->connect_call_count(), connects);
}

}  // namespace
}  // namespace rh56f1_hardware
