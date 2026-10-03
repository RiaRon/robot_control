// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// rh56f1_diag: read-only raw snapshot of one RH56F1 hand (Stage 5B-2b).
//
// - Dry run by default: prints the exact request frames and never opens the
//   port. Opening needs --execute-read-only.
// - Only read requests (cmd 0x11) of the registers in kDiagRegisters are ever
//   built. The tool has no code path that writes any register.
// - Port, device ID and baud are required; there is no ID scan.
// - One snapshot by default; repetition needs --samples N plus --period-sec,
//   both bounded (kMaxSamples, kMinPeriodSec..kMaxPeriodSec).
// - Values are printed in vendor raw units only; no radian conversion.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "rh56f1_hardware/protocol.hpp"
#include "rh56f1_hardware/serial_port.hpp"

namespace rh56f1_hardware::diag {

struct DiagRegister {
  const char* name;
  std::uint16_t address;
  const char* unit;
};

// All six-DOF, read-only groups (manual table 30).
inline constexpr DiagRegister kDiagRegisters[] = {
    {"ANGLE_ACT", protocol::kRegAngleAct, "0.1 deg (vendor angle)"},
    {"FORCE_ACT", protocol::kRegForceAct, "g"},
    {"CURRENT_ACT", protocol::kRegCurrentAct, "mA"},
    {"ERROR", protocol::kRegErrorCode, "bit field"},
    {"STATUS", protocol::kRegStatus, "status code"},
    {"TEMP", protocol::kRegTemperature, "deg C"},
};

// Tool limits chosen for a first read-only check. They are NOT vendor values
// (the vendor publishes no poll rate or reply timeout).
inline constexpr int kMaxSamples = 20;
inline constexpr double kMinPeriodSec = 0.5;
inline constexpr double kMaxPeriodSec = 10.0;
// Diagnostic wait for one reply; generous against the vendor SDK's 25 ms.
inline constexpr int kDefaultReplyTimeoutMs = 100;
inline constexpr int kMinReplyTimeoutMs = 10;
inline constexpr int kMaxReplyTimeoutMs = 1000;

inline constexpr const char* kByIdPrefix = "/dev/serial/by-id/";

struct DiagOptions {
  std::string side;  // "left" or "right"
  std::string port;
  int device_id = -1;
  int baud = 0;
  bool execute_read_only = false;
  int samples = 1;
  double period_sec = 0.0;
  int reply_timeout_ms = kDefaultReplyTimeoutMs;
  // Test/bench escape hatch for non-by-id paths (e.g. a PTY). Documented as
  // not for use with the robot.
  bool allow_non_by_id_port = false;
};

// Returns false with a message for any missing/invalid/conflicting option.
bool parse_args(const std::vector<std::string>& args, DiagOptions& out, std::string& error);

std::string usage();

// Opens the port for --execute-read-only. Replaceable in tests.
using PortOpener =
    std::function<std::unique_ptr<ByteStream>(const DiagOptions&, std::string& error)>;
PortOpener default_port_opener();

// The warning printed before any real read.
extern const char* const kPowerOnWarning;

enum ExitCode : int {
  kExitOk = 0,
  kExitUsage = 2,
  kExitOpenFailed = 3,
  kExitReadFailed = 4,
};

// Runs one diagnostic session. `sleep` is injectable for tests.
int run(const DiagOptions& options, std::ostream& out, const PortOpener& opener,
        const std::function<void(std::chrono::milliseconds)>& sleep = {});

}  // namespace rh56f1_hardware::diag
