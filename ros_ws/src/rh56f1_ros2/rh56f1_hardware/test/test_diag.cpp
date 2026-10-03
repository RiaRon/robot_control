// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// rh56f1_diag: argument checks, dry run never opens a port, and an
// --execute-read-only run against the PTY fake sends only read requests.
// The by-id strings below are syntax examples; nothing opens them.

#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "fake_rh56f1.hpp"
#include "gmock/gmock.h"
#include "rh56f1_hardware/diag.hpp"

namespace rh56f1_hardware::diag {
namespace {

using ::testing::HasSubstr;
using ::testing::Not;

const std::string kExamplePort = "/dev/serial/by-id/usb-EXAMPLE-if00-port0";

std::vector<std::string> Base() {
  return {"--side", "left", "--port", kExamplePort, "--device-id", "1", "--baud", "115200"};
}

std::vector<std::string> With(std::vector<std::string> args, std::vector<std::string> extra) {
  args.insert(args.end(), extra.begin(), extra.end());
  return args;
}

bool Parses(const std::vector<std::string>& args, DiagOptions* out = nullptr,
            std::string* error = nullptr) {
  DiagOptions o;
  std::string e;
  const bool ok = parse_args(args, o, e);
  if (out) *out = o;
  if (error) *error = e;
  return ok;
}

std::vector<std::string> Without(std::vector<std::string> args, const std::string& flag) {
  for (std::size_t i = 0; i < args.size(); ++i) {
    if (args[i] == flag) {
      args.erase(args.begin() + static_cast<long>(i), args.begin() + static_cast<long>(i) + 2);
      break;
    }
  }
  return args;
}

TEST(DiagArgs, DefaultsToDryRunSingleSnapshot) {
  DiagOptions o;
  ASSERT_TRUE(Parses(Base(), &o));
  EXPECT_FALSE(o.execute_read_only);
  EXPECT_EQ(o.samples, 1);
  EXPECT_EQ(o.device_id, 1);
  EXPECT_EQ(o.baud, 115200);
}

TEST(DiagArgs, PortIdBaudAndSideAreRequired) {
  for (const char* flag : {"--side", "--port", "--device-id", "--baud"}) {
    std::string error;
    EXPECT_FALSE(Parses(Without(Base(), flag), nullptr, &error)) << flag;
    EXPECT_FALSE(error.empty());
  }
}

TEST(DiagArgs, RejectsInvalidValues) {
  auto set = [](const std::string& flag, const std::string& value) {
    auto args = Base();
    for (std::size_t i = 0; i + 1 < args.size(); ++i) {
      if (args[i] == flag) args[i + 1] = value;
    }
    return args;
  };
  EXPECT_FALSE(Parses(set("--side", "both")));
  EXPECT_FALSE(Parses(set("--device-id", "0")));
  EXPECT_FALSE(Parses(set("--device-id", "255")));
  EXPECT_FALSE(Parses(set("--device-id", "1x")));
  EXPECT_FALSE(Parses(set("--baud", "9600")));
  EXPECT_FALSE(Parses(set("--port", "/dev/ttyUSB0")));  // unstable name refused
  EXPECT_FALSE(Parses(With(Base(), {"--scan"})));
  EXPECT_FALSE(Parses(With(Base(), {"--dry-run", "--execute-read-only"})));
}

TEST(DiagArgs, RepetitionNeedsBoundedSamplesAndPeriod) {
  EXPECT_FALSE(Parses(With(Base(), {"--samples", "5"})));
  EXPECT_TRUE(Parses(With(Base(), {"--samples", "5", "--period-sec", "1.0"})));
  EXPECT_FALSE(Parses(With(Base(), {"--samples", "0"})));
  EXPECT_FALSE(Parses(With(Base(), {"--samples", std::to_string(kMaxSamples + 1),
                                    "--period-sec", "1.0"})));
  EXPECT_FALSE(Parses(With(Base(), {"--samples", "5", "--period-sec", "0.1"})));
  EXPECT_FALSE(Parses(With(Base(), {"--samples", "5", "--period-sec", "nan"})));
  EXPECT_FALSE(Parses(With(Base(), {"--period-sec", "1.0"})));
  EXPECT_FALSE(Parses(With(Base(), {"--reply-timeout-ms", "5"})));
}

TEST(DiagRun, DryRunNeverOpensThePortAndPrintsReadFramesOnly) {
  DiagOptions o;
  ASSERT_TRUE(Parses(Base(), &o));
  int opens = 0;
  PortOpener opener = [&](const DiagOptions&, std::string&) -> std::unique_ptr<ByteStream> {
    ++opens;
    return nullptr;
  };
  std::ostringstream out;
  EXPECT_EQ(run(o, out, opener), kExitOk);
  EXPECT_EQ(opens, 0);
  const std::string text = out.str();
  EXPECT_THAT(text, HasSubstr("DRY RUN"));
  EXPECT_THAT(text, HasSubstr("EB 90 01 04 11 28 04 0C 4E"));  // ANGLE_ACT, manual table 6
  EXPECT_THAT(text, HasSubstr("no software can guarantee"));
  EXPECT_THAT(text, Not(HasSubstr(" 12 ")));  // no write command byte in any frame
}

TEST(DiagRun, OpenFailureIsReported) {
  DiagOptions o;
  ASSERT_TRUE(Parses(With(Base(), {"--execute-read-only"}), &o));
  PortOpener opener = [](const DiagOptions&, std::string& error) -> std::unique_ptr<ByteStream> {
    error = "open_failed (errno 2)";
    return nullptr;
  };
  std::ostringstream out;
  EXPECT_EQ(run(o, out, opener), kExitOpenFailed);
}

TEST(DiagRun, ExecuteReadOnlyAgainstPtySendsOnlyReads) {
  testing::FakeRh56f1 fake(7);
  ASSERT_TRUE(fake.ok());
  fake.set_register(protocol::kRegAngleAct, 1740);
  fake.set_register(protocol::kRegErrorCode + 3, 0x06);
  fake.set_register(protocol::kRegStatus + 1, 2);
  DiagOptions o;
  ASSERT_TRUE(Parses({"--side", "right", "--port", fake.slave_path(), "--allow-non-by-id-port",
                      "--device-id", "7", "--baud", "115200", "--execute-read-only",
                      "--samples", "2", "--period-sec", "0.5"},
                     &o));
  std::ostringstream out;
  int sleeps = 0;
  const int code = run(o, out, default_port_opener(),
                       [&](std::chrono::milliseconds) { ++sleeps; });
  EXPECT_EQ(code, kExitOk) << out.str();
  EXPECT_EQ(sleeps, 1);
  EXPECT_EQ(fake.write_requests(), 0u);
  EXPECT_EQ(fake.read_requests(), 2u * std::size(kDiagRegisters));
  for (const auto& req : fake.requests()) EXPECT_EQ(req[4], 0x11);
  const std::string text = out.str();
  EXPECT_THAT(text, HasSubstr("little         1740"));
  EXPECT_THAT(text, HasSubstr("[over_temperature|over_current]"));
  EXPECT_THAT(text, HasSubstr("[position_reached]"));
  EXPECT_THAT(text, HasSubstr("writes sent: 0"));
  EXPECT_THAT(text, HasSubstr("no radian conversion"));
}

TEST(DiagRun, SilentDeviceIsAReadFailureNotAHang) {
  testing::FakeRh56f1 fake(7);
  fake.set_mode(testing::FakeRh56f1::Mode::kSilent);
  DiagOptions o;
  ASSERT_TRUE(Parses({"--side", "left", "--port", fake.slave_path(), "--allow-non-by-id-port",
                      "--device-id", "7", "--baud", "115200", "--execute-read-only",
                      "--reply-timeout-ms", "20"},
                     &o));
  std::ostringstream out;
  EXPECT_EQ(run(o, out, default_port_opener()), kExitReadFailed);
  EXPECT_THAT(out.str(), HasSubstr("status   : timeout"));
  EXPECT_EQ(fake.write_requests(), 0u);
}

}  // namespace
}  // namespace rh56f1_hardware::diag
