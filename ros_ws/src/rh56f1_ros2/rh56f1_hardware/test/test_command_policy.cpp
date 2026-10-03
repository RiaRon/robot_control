// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <limits>

#include "gmock/gmock.h"
#include "rh56f1_hardware/command_policy.hpp"

namespace rh56f1_hardware {
namespace {

using Verdict = CommandPolicy::Verdict;

ActuatorTargets Filled(double value) {
  ActuatorTargets t;
  t.fill(value);
  return t;
}

PolicyConfig Config(double velocity = 2.0, double step = 0.02, double period = 0.1) {
  PolicyConfig config;
  config.limits.fill(JointLimits{0.0, 1.5});
  config.max_velocity_rad_s = velocity;
  config.max_step_rad = step;
  config.min_write_period_sec = period;
  return config;
}

TEST(PolicyConfig, ValidatesEveryField) {
  EXPECT_EQ(validate(Config()), "");
  auto bad = Config();
  bad.limits[2] = {1.0, 1.0};
  EXPECT_THAT(validate(bad), ::testing::HasSubstr("actuator 2"));
  EXPECT_NE(validate(Config(0.0)), "");
  EXPECT_NE(validate(Config(2.0, 0.0)), "");
  EXPECT_NE(validate(Config(2.0, 0.02, -1.0)), "");
  EXPECT_NE(validate(Config(std::numeric_limits<double>::infinity())), "");
}

TEST(CommandPolicy, NothingIsSentBeforeSeeding) {
  CommandPolicy policy(Config());
  const auto decision = policy.decide(Filled(0.5), 1.0);
  EXPECT_EQ(decision.verdict, Verdict::kNotSeeded);
}

TEST(CommandPolicy, SmallMoveWithinStepIsSentExactly) {
  CommandPolicy policy(Config());
  policy.reset(Filled(0.5), 0.0);
  const auto decision = policy.decide(Filled(0.51), 0.2);
  ASSERT_EQ(decision.verdict, Verdict::kSend);
  for (double v : decision.targets) EXPECT_DOUBLE_EQ(v, 0.51);
  EXPECT_THAT(decision.step_limited, ::testing::Each(false));
}

TEST(CommandPolicy, LargeMoveIsLimitedToOneStepPerSend) {
  CommandPolicy policy(Config(2.0, 0.02, 0.1));
  policy.reset(Filled(0.5), 0.0);
  auto decision = policy.decide(Filled(1.0), 0.1);
  ASSERT_EQ(decision.verdict, Verdict::kSend);
  for (double v : decision.targets) EXPECT_DOUBLE_EQ(v, 0.52);
  EXPECT_THAT(decision.step_limited, ::testing::Each(true));
  decision = policy.decide(Filled(1.0), 0.2);
  for (double v : decision.targets) EXPECT_DOUBLE_EQ(v, 0.54);
}

TEST(CommandPolicy, VelocityBoundsTheStepWhenSendsAreClose) {
  // 2 rad/s over 5 ms allows 0.01 rad, tighter than the 0.02 rad step.
  CommandPolicy policy(Config(2.0, 0.02, 0.0));
  policy.reset(Filled(0.5), 0.0);
  const auto decision = policy.decide(Filled(1.0), 0.005);
  ASSERT_EQ(decision.verdict, Verdict::kSend);
  for (double v : decision.targets) EXPECT_NEAR(v, 0.51, 1e-12);
}

TEST(CommandPolicy, SendsFasterThanTheWritePeriodAreRateLimited) {
  CommandPolicy policy(Config(2.0, 0.02, 0.1));
  policy.reset(Filled(0.5), 0.0);
  ASSERT_EQ(policy.decide(Filled(0.5), 0.1).verdict, Verdict::kSend);
  EXPECT_EQ(policy.decide(Filled(0.6), 0.15).verdict, Verdict::kRateLimited);
  EXPECT_EQ(policy.decide(Filled(0.6), 0.2).verdict, Verdict::kSend);
}

TEST(CommandPolicy, FiniteOutOfLimitIsClampedAndReportedPerJoint) {
  CommandPolicy policy(Config(2.0, 5.0, 0.0));
  policy.reset(Filled(0.5), 0.0);
  ActuatorTargets requested = Filled(0.5);
  requested[1] = 3.0;
  requested[4] = -1.0;
  const auto decision = policy.decide(requested, 10.0);
  ASSERT_EQ(decision.verdict, Verdict::kSend);
  EXPECT_TRUE(decision.position_clamped[1]);
  EXPECT_TRUE(decision.position_clamped[4]);
  EXPECT_FALSE(decision.position_clamped[0]);
  EXPECT_DOUBLE_EQ(decision.requested[1], 3.0);
  EXPECT_DOUBLE_EQ(decision.clamped_to[1], 1.5);
  EXPECT_DOUBLE_EQ(decision.clamped_to[4], 0.0);
  EXPECT_DOUBLE_EQ(decision.targets[1], 1.5);
  EXPECT_DOUBLE_EQ(decision.targets[4], 0.0);
}

TEST(CommandPolicy, NonFiniteIsRejectedNotClamped) {
  CommandPolicy policy(Config());
  policy.reset(Filled(0.5), 0.0);
  for (double bad : {std::numeric_limits<double>::quiet_NaN(),
                     std::numeric_limits<double>::infinity(),
                     -std::numeric_limits<double>::infinity()}) {
    ActuatorTargets requested = Filled(0.5);
    requested[3] = bad;
    const auto decision = policy.decide(requested, 1.0);
    EXPECT_EQ(decision.verdict, Verdict::kRejectedNonFinite);
    EXPECT_THAT(decision.position_clamped, ::testing::Each(false));
  }
  EXPECT_THAT(policy.last_sent(), ::testing::Each(0.5));
}

TEST(CommandPolicy, UnseedStopsSending) {
  CommandPolicy policy(Config());
  policy.reset(Filled(0.5), 0.0);
  policy.unseed();
  EXPECT_EQ(policy.decide(Filled(0.5), 1.0).verdict, Verdict::kNotSeeded);
}

}  // namespace
}  // namespace rh56f1_hardware
