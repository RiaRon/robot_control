// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

// ROS- and CAN-free decisions OpenArmHW makes around activation and state
// freshness, kept header-only so they are unit-testable without a SocketCAN
// device (test/test_startup_safety.cpp).
namespace openarm_hardware::startup_safety {

// true/1/yes and false/0/no, case-insensitive; anything else is rejected
// rather than silently read as false.
inline std::optional<bool> parse_bool(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (text == "true" || text == "1" || text == "yes") return true;
  if (text == "false" || text == "0" || text == "no") return false;
  return std::nullopt;
}

struct Bounds {
  double lower = -std::numeric_limits<double>::infinity();
  double upper = std::numeric_limits<double>::infinity();
};

struct Verdict {
  bool ok = false;
  std::string reason;
};

// A motor's state counts as measured only if its receive counter advanced
// after `before` was taken: the CAN library initializes every position to 0.0
// and has no validity flag, so a value that was never received must not be
// mistaken for a measurement. A received value must also be finite and within
// its bounds widened by `margin` (droop against a stop), else it is refused.
inline Verdict check_fresh_and_valid(const std::vector<uint64_t>& before,
                                     const std::vector<uint64_t>& after,
                                     const std::vector<double>& positions,
                                     const std::vector<Bounds>& bounds,
                                     const std::vector<std::string>& names,
                                     double margin) {
  const size_t n = names.size();
  if (before.size() != n || after.size() != n || positions.size() != n ||
      bounds.size() != n) {
    return {false, "internal size mismatch in fresh-state check"};
  }
  std::string missing;
  for (size_t i = 0; i < n; ++i) {
    if (after[i] <= before[i]) {
      missing += (missing.empty() ? "" : ", ") + names[i];
    }
  }
  if (!missing.empty()) {
    return {false, "no fresh state received from: " + missing};
  }
  for (size_t i = 0; i < n; ++i) {
    const double q = positions[i];
    if (!std::isfinite(q)) {
      return {false, names[i] + " reported a non-finite position"};
    }
    if (q < bounds[i].lower - margin || q > bounds[i].upper + margin) {
      return {false, names[i] + " measured " + std::to_string(q) +
                         " rad, outside [" + std::to_string(bounds[i].lower) +
                         ", " + std::to_string(bounds[i].upper) + "] by more than " +
                         std::to_string(margin) + " rad"};
    }
  }
  return {true, ""};
}

// Explicit-activation gate: activation is refused until the component has sat
// configured-but-inactive for at least `min_inactive_sec`. controller_manager
// auto-activates components immediately after configuring them when no
// hardware_components_initial_state is given, so this keeps a description
// started outside its launch file from enabling motors on its own.
// min_inactive_sec <= 0 disables the gate (the stock OpenArm bringup).
inline Verdict activation_dwell(double configured_at_sec, double now_sec,
                                double min_inactive_sec) {
  if (min_inactive_sec <= 0.0) return {true, ""};
  if (!std::isfinite(configured_at_sec)) {
    return {false, "component was never configured"};
  }
  const double inactive_for = now_sec - configured_at_sec;
  if (inactive_for < min_inactive_sec) {
    return {false, "activation requested " + std::to_string(inactive_for) +
                       " s after configure; explicit activation requires at least " +
                       std::to_string(min_inactive_sec) +
                       " s inactive (auto-activation at startup is refused)"};
  }
  return {true, ""};
}

// Per-motor last-fresh bookkeeping for stale-state detection in read().
class StaleMonitor {
 public:
  void reset(const std::vector<uint64_t>& counts, double now_sec) {
    last_counts_ = counts;
    last_fresh_sec_.assign(counts.size(), now_sec);
  }

  void observe(const std::vector<uint64_t>& counts, double now_sec) {
    if (counts.size() != last_counts_.size()) {
      reset(counts, now_sec);
      return;
    }
    for (size_t i = 0; i < counts.size(); ++i) {
      if (counts[i] != last_counts_[i]) {
        last_counts_[i] = counts[i];
        last_fresh_sec_[i] = now_sec;
      }
    }
  }

  // Index of the first motor with no fresh state for longer than timeout_sec,
  // or -1. timeout_sec <= 0 disables detection.
  int first_stale(double now_sec, double timeout_sec) const {
    if (timeout_sec <= 0.0) return -1;
    for (size_t i = 0; i < last_fresh_sec_.size(); ++i) {
      if (now_sec - last_fresh_sec_[i] > timeout_sec) return static_cast<int>(i);
    }
    return -1;
  }

 private:
  std::vector<uint64_t> last_counts_;
  std::vector<double> last_fresh_sec_;
};

}  // namespace openarm_hardware::startup_safety
