// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/command_policy.hpp"

#include <algorithm>
#include <cmath>

namespace rh56f1_hardware {

std::string validate(const PolicyConfig& config) {
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const auto& limit = config.limits[i];
    if (!std::isfinite(limit.lower) || !std::isfinite(limit.upper) ||
        !(limit.lower < limit.upper)) {
      return "actuator " + std::to_string(i) + " has invalid limits";
    }
  }
  if (!std::isfinite(config.max_velocity_rad_s) || config.max_velocity_rad_s <= 0.0) {
    return "max_velocity_rad_s must be a positive number";
  }
  if (!std::isfinite(config.max_step_rad) || config.max_step_rad <= 0.0) {
    return "max_step_rad must be a positive number";
  }
  if (!std::isfinite(config.min_write_period_sec) || config.min_write_period_sec < 0.0) {
    return "min_write_period_sec must be a non-negative number";
  }
  return "";
}

CommandPolicy::CommandPolicy(PolicyConfig config) : config_(config) {}

void CommandPolicy::reset(const ActuatorTargets& current, double now_sec) {
  last_sent_ = current;
  last_send_sec_ = now_sec;
  seeded_ = true;
}

CommandPolicy::Decision CommandPolicy::decide(const ActuatorTargets& requested,
                                              double now_sec) {
  Decision decision;
  decision.requested = requested;
  decision.targets = last_sent_;
  if (!seeded_) {
    decision.verdict = Verdict::kNotSeeded;
    return decision;
  }
  const bool finite = std::all_of(requested.begin(), requested.end(),
                                  [](double v) { return std::isfinite(v); });
  if (!finite) {
    decision.verdict = Verdict::kRejectedNonFinite;
    return decision;
  }

  ActuatorTargets desired{};
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    desired[i] = std::clamp(requested[i], config_.limits[i].lower, config_.limits[i].upper);
    decision.clamped_to[i] = desired[i];
    decision.position_clamped[i] = desired[i] != requested[i];
  }

  const double elapsed = now_sec - last_send_sec_;
  if (elapsed < config_.min_write_period_sec) {
    decision.verdict = Verdict::kRateLimited;
    return decision;
  }
  const double max_delta =
      std::min(config_.max_velocity_rad_s * std::max(elapsed, 0.0), config_.max_step_rad);
  for (std::size_t i = 0; i < kActuatorCount; ++i) {
    const double delta = desired[i] - last_sent_[i];
    const double limited = std::clamp(delta, -max_delta, max_delta);
    decision.step_limited[i] = limited != delta;
    decision.targets[i] = last_sent_[i] + limited;
  }
  decision.verdict = Verdict::kSend;
  last_sent_ = decision.targets;
  last_send_sec_ = now_sec;
  return decision;
}

}  // namespace rh56f1_hardware
