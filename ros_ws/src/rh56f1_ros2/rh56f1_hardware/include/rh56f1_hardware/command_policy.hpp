// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <array>
#include <string>

#include "rh56f1_hardware/transport.hpp"

namespace rh56f1_hardware {

struct JointLimits {
  double lower = 0.0;
  double upper = 0.0;
};

using JointLimitsArray = std::array<JointLimits, kActuatorCount>;

struct PolicyConfig {
  JointLimitsArray limits{};
  // Both > 0; the per-send step is min(max_velocity * elapsed, max_step).
  double max_velocity_rad_s = 0.0;
  double max_step_rad = 0.0;
  double min_write_period_sec = 0.0;
};

// Empty when valid, else what is wrong.
std::string validate(const PolicyConfig& config);

// ROS-free per-send decision for one hand's six actuators. A finite command
// outside a joint's limits is clamped to them and reported per joint; a
// non-finite command is rejected, never clamped. Every sent command moves at
// most one step from the previously sent one.
class CommandPolicy {
 public:
  explicit CommandPolicy(PolicyConfig config);

  enum class Verdict {
    kSend,
    kRateLimited,
    kRejectedNonFinite,
    kNotSeeded,
  };

  struct Decision {
    Verdict verdict = Verdict::kNotSeeded;
    ActuatorTargets targets{};
    ActuatorTargets requested{};
    std::array<bool, kActuatorCount> position_clamped{};
    ActuatorTargets clamped_to{};
    std::array<bool, kActuatorCount> step_limited{};
  };

  // `current`: where the device is now. Until this is called nothing is sent.
  void reset(const ActuatorTargets& current, double now_sec);
  void unseed() { seeded_ = false; }

  Decision decide(const ActuatorTargets& requested, double now_sec);

  const ActuatorTargets& last_sent() const { return last_sent_; }
  const PolicyConfig& config() const { return config_; }

 private:
  PolicyConfig config_;
  ActuatorTargets last_sent_{};
  double last_send_sec_ = 0.0;
  bool seeded_ = false;
};

}  // namespace rh56f1_hardware
