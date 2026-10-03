// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "openarm_hardware/startup_safety.hpp"

namespace ss = openarm_hardware::startup_safety;

TEST(ParseBool, AcceptsTheSixSpellingsCaseInsensitively) {
  for (const char* text : {"true", "TRUE", "True", "1", "yes", "YES"}) {
    ASSERT_TRUE(ss::parse_bool(text).has_value()) << text;
    EXPECT_TRUE(*ss::parse_bool(text)) << text;
  }
  for (const char* text : {"false", "FALSE", "0", "no", "No"}) {
    ASSERT_TRUE(ss::parse_bool(text).has_value()) << text;
    EXPECT_FALSE(*ss::parse_bool(text)) << text;
  }
}

TEST(ParseBool, RejectsAnythingElseInsteadOfReadingItAsFalse) {
  for (const char* text : {"", "on", "off", "2", "truee", " true"}) {
    EXPECT_FALSE(ss::parse_bool(text).has_value()) << "'" << text << "'";
  }
}

namespace {
const std::vector<std::string> kNames = {"j1", "j2"};
const std::vector<ss::Bounds> kBounds = {{-1.0, 1.0}, {0.0, 2.0}};
}  // namespace

TEST(FreshState, NeverReceivedInitialZeroIsNotAMeasurement) {
  // Both motors still hold the library's 0.0 initial value, which lies inside
  // their bounds; only the unchanged receive counter reveals it was never read.
  const auto verdict =
      ss::check_fresh_and_valid({0, 0}, {1, 0}, {0.0, 0.0}, kBounds, kNames, 0.05);
  EXPECT_FALSE(verdict.ok);
  EXPECT_NE(verdict.reason.find("j2"), std::string::npos);
  EXPECT_EQ(verdict.reason.find("j1"), std::string::npos);
}

TEST(FreshState, AllReceivedAndInBoundsIsAccepted) {
  EXPECT_TRUE(ss::check_fresh_and_valid({4, 9}, {5, 10}, {0.3, 1.2}, kBounds, kNames, 0.05).ok);
}

TEST(FreshState, NonFiniteMeasurementIsRefused) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ss::check_fresh_and_valid({0, 0}, {1, 1}, {nan, 1.0}, kBounds, kNames, 0.05).ok);
  EXPECT_FALSE(ss::check_fresh_and_valid({0, 0}, {1, 1}, {0.0, inf}, kBounds, kNames, 0.05).ok);
}

TEST(FreshState, OutOfRangeBeyondMarginIsRefusedWithinMarginAccepted) {
  EXPECT_TRUE(ss::check_fresh_and_valid({0, 0}, {1, 1}, {1.04, -0.04}, kBounds, kNames, 0.05).ok);
  const auto verdict =
      ss::check_fresh_and_valid({0, 0}, {1, 1}, {1.06, 1.0}, kBounds, kNames, 0.05);
  EXPECT_FALSE(verdict.ok);
  EXPECT_NE(verdict.reason.find("j1"), std::string::npos);
}

TEST(FreshState, UnboundedJointsOnlyNeedFiniteness) {
  const std::vector<ss::Bounds> open(2);
  EXPECT_TRUE(ss::check_fresh_and_valid({0, 0}, {1, 1}, {100.0, -100.0}, open, kNames, 0.0).ok);
}

TEST(FreshState, SizeMismatchIsRefused) {
  EXPECT_FALSE(ss::check_fresh_and_valid({0}, {1, 1}, {0.0, 0.0}, kBounds, kNames, 0.05).ok);
}

TEST(ActivationDwell, DisabledWhenMinimumIsZero) {
  EXPECT_TRUE(ss::activation_dwell(10.0, 10.0, 0.0).ok);
  EXPECT_TRUE(ss::activation_dwell(std::nan(""), 10.0, 0.0).ok);
}

TEST(ActivationDwell, ImmediateAutoActivationIsRefused) {
  const auto verdict = ss::activation_dwell(10.0, 10.001, 2.0);
  EXPECT_FALSE(verdict.ok);
  EXPECT_NE(verdict.reason.find("auto-activation"), std::string::npos);
}

TEST(ActivationDwell, ExplicitActivationAfterDwellIsAllowed) {
  EXPECT_TRUE(ss::activation_dwell(10.0, 12.5, 2.0).ok);
}

TEST(ActivationDwell, NeverConfiguredIsRefused) {
  EXPECT_FALSE(ss::activation_dwell(std::nan(""), 12.5, 2.0).ok);
}

TEST(StaleMonitor, AdvancingCountersStayFresh) {
  ss::StaleMonitor monitor;
  monitor.reset({0, 0}, 0.0);
  monitor.observe({1, 1}, 0.1);
  monitor.observe({2, 2}, 0.2);
  EXPECT_EQ(monitor.first_stale(0.25, 0.1), -1);
}

TEST(StaleMonitor, OneSilentMotorIsReportedByIndex) {
  ss::StaleMonitor monitor;
  monitor.reset({0, 0}, 0.0);
  monitor.observe({1, 0}, 0.05);
  monitor.observe({2, 0}, 0.15);
  EXPECT_EQ(monitor.first_stale(0.15, 0.1), 1);
}

TEST(StaleMonitor, ZeroTimeoutDisablesDetection) {
  ss::StaleMonitor monitor;
  monitor.reset({0}, 0.0);
  EXPECT_EQ(monitor.first_stale(1000.0, 0.0), -1);
}
