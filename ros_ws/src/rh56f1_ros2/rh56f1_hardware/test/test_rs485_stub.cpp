// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "rh56f1_hardware/backend.hpp"
#include "rh56f1_hardware/rs485_transport.hpp"

namespace rh56f1_hardware {
namespace {

TEST(Rs485Stub, EveryOperationRefusesAndNothingReportsConnected) {
  Rs485Transport transport("/dev/null", 115200, 1);
  EXPECT_FALSE(transport.connect());
  EXPECT_FALSE(transport.is_connected());
  EXPECT_EQ(transport.last_error(), TransportError::kNotImplemented);
  ActuatorTargets targets{};
  EXPECT_FALSE(transport.write_positions(targets));
  ActuatorStates states{};
  EXPECT_FALSE(transport.read_state(states));
  EXPECT_THAT(transport.describe(), ::testing::HasSubstr("not implemented"));
}

TEST(Rs485Stub, BackendCannotConnectArmOrSendThroughIt) {
  BackendConfig config;
  config.policy.limits.fill(JointLimits{0.0, 1.0});
  config.policy.max_velocity_rad_s = 2.0;
  config.policy.max_step_rad = 0.02;
  config.policy.min_write_period_sec = 0.1;
  config.stale_timeout_sec = 0.25;
  config.state_poll_period_sec = 0.02;
  config.min_fresh_reads_to_arm = 1;
  config.min_inactive_sec_before_arm = 0.0;
  Rh56f1Backend backend(std::make_unique<Rs485Transport>("/dev/null", 115200, 1), config);

  std::string why;
  EXPECT_FALSE(backend.connect(0.0, &why));
  EXPECT_THAT(why, ::testing::HasSubstr("not_implemented"));
  for (int i = 0; i < 10; ++i) backend.step(0.1 * i);
  EXPECT_FALSE(backend.arm(10.0, &why));
  const auto snapshot = backend.snapshot();
  EXPECT_FALSE(snapshot.connected);
  EXPECT_FALSE(snapshot.armed);
  EXPECT_EQ(snapshot.writes, 0u);
  EXPECT_EQ(snapshot.good_reads, 0u);
}

}  // namespace
}  // namespace rh56f1_hardware
