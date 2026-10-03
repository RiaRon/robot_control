// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// SerialPort + RegisterClient against a PTY-backed fake RH56F1. No real
// device path is referenced anywhere in this file.

#include <dirent.h>

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "fake_rh56f1.hpp"
#include "gmock/gmock.h"
#include "rh56f1_hardware/register_client.hpp"
#include "rh56f1_hardware/rs485_codec.hpp"
#include "rh56f1_hardware/serial_port.hpp"

namespace rh56f1_hardware {
namespace {

using testing::FakeRh56f1;
using Mode = FakeRh56f1::Mode;
using ::testing::ElementsAre;
using std::chrono::milliseconds;

constexpr int kBaud = 115200;
constexpr std::uint8_t kId = 3;
constexpr milliseconds kReplyTimeout{150};

std::size_t OpenFdCount() {
  std::size_t count = 0;
  if (DIR* dir = ::opendir("/proc/self/fd")) {
    while (::readdir(dir) != nullptr) ++count;
    ::closedir(dir);
  }
  return count;
}

void LoadAngles(FakeRh56f1& fake) {
  const std::array<std::int16_t, 6> angles = {900, 1000, 1100, 1200, 1300, 1400};
  for (std::size_t i = 0; i < angles.size(); ++i) {
    fake.set_register(static_cast<std::uint16_t>(protocol::kRegAngleAct + i), angles[i]);
  }
}

struct Rig {
  explicit Rig(std::uint8_t id = kId) : fake(id) {
    LoadAngles(fake);
    status = port.open(fake.slave_path(), kBaud);
  }
  FakeRh56f1 fake;
  SerialPort port;
  SerialStatus status;
};

Transaction ReadAngles(RegisterClient& client) {
  return client.read(protocol::kRegAngleAct, protocol::kDofGroupBytes);
}

void ExpectAngles(const Transaction& t) {
  ASSERT_EQ(t.status, ClientStatus::kOk) << to_string(t.status);
  std::array<std::int16_t, protocol::kDofCount> v{};
  ASSERT_TRUE(rs485::decode_dof_group(t.payload, v));
  EXPECT_THAT(v, ElementsAre(900, 1000, 1100, 1200, 1300, 1400));
}

// ---- SerialPort basics ---------------------------------------------------------

TEST(SerialPort, RejectsUnsupportedBaudAndEmptyPath) {
  SerialPort port;
  EXPECT_EQ(port.open("", kBaud), SerialStatus::kInvalidArgument);
  FakeRh56f1 fake(kId);
  EXPECT_EQ(port.open(fake.slave_path(), 9600), SerialStatus::kInvalidArgument);
  EXPECT_EQ(port.open(fake.slave_path(), 1000000), SerialStatus::kInvalidArgument);
  EXPECT_FALSE(port.is_open());
  for (int baud : {19200, 57600, 115200, 921600}) EXPECT_TRUE(is_supported_rh56f1_baud(baud));
}

TEST(SerialPort, MissingPathFailsAndLeaksNothing) {
  const std::size_t before = OpenFdCount();
  SerialPort port;
  EXPECT_EQ(port.open("/nonexistent/rh56f1-test-tty", kBaud), SerialStatus::kOpenFailed);
  EXPECT_EQ(OpenFdCount(), before);
}

TEST(SerialPort, OperationsOnClosedPortReportNotOpen) {
  SerialPort port;
  std::vector<std::uint8_t> buf;
  EXPECT_EQ(port.write_all({0x01}, milliseconds(10)), SerialStatus::kNotOpen);
  EXPECT_EQ(port.read_some(buf, milliseconds(10)), SerialStatus::kNotOpen);
  EXPECT_EQ(port.discard_input(), 0u);
}

TEST(SerialPort, SecondOwnerIsRefusedAndOwnershipIsReleasedOnClose) {
  FakeRh56f1 fake(kId);
  SerialPort first;
  ASSERT_EQ(first.open(fake.slave_path(), kBaud), SerialStatus::kOk);
  SerialPort second;
  EXPECT_EQ(second.open(fake.slave_path(), kBaud), SerialStatus::kPortBusy);
  first.close();
  EXPECT_EQ(second.open(fake.slave_path(), kBaud), SerialStatus::kOk);
}

TEST(SerialPort, OpenAndIdleSendNothing) {
  Rig rig;
  ASSERT_EQ(rig.status, SerialStatus::kOk);
  RegisterClient client(rig.port, kId, kReplyTimeout);
  std::this_thread::sleep_for(milliseconds(100));
  EXPECT_EQ(rig.fake.bytes_received(), 0u);
  EXPECT_EQ(client.write_requests_sent(), 0u);
  EXPECT_EQ(client.read_requests_sent(), 0u);
}

TEST(SerialPort, ReadTimesOutWithoutData) {
  Rig rig;
  std::vector<std::uint8_t> buf;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(rig.port.read_some(buf, milliseconds(50)), SerialStatus::kTimeout);
  EXPECT_GE(std::chrono::steady_clock::now() - start, milliseconds(45));
}

TEST(SerialPort, ClosesItsFdOnDestruction) {
  FakeRh56f1 fake(kId);
  const std::size_t before = OpenFdCount();
  {
    SerialPort port;
    ASSERT_EQ(port.open(fake.slave_path(), kBaud), SerialStatus::kOk);
    EXPECT_EQ(OpenFdCount(), before + 1);
  }
  EXPECT_EQ(OpenFdCount(), before);
}

// ---- RegisterClient over the PTY ------------------------------------------------

TEST(RegisterClientPty, NormalReadSendsTheExactOfficialFrame) {
  Rig rig(1);
  ASSERT_EQ(rig.status, SerialStatus::kOk);
  RegisterClient client(rig.port, 1, kReplyTimeout);
  const Transaction t = ReadAngles(client);
  ExpectAngles(t);
  // Manual table 6 / example 9.
  EXPECT_THAT(t.request, ElementsAre(0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C, 0x4E));
  ASSERT_EQ(rig.fake.requests().size(), 1u);
  EXPECT_EQ(rig.fake.requests()[0], t.request);
  EXPECT_EQ(t.frames_rejected, 0u);
}

TEST(RegisterClientPty, FragmentedReply) {
  Rig rig;
  rig.fake.set_mode(Mode::kFragmented);
  RegisterClient client(rig.port, kId, kReplyTimeout);
  ExpectAngles(ReadAngles(client));
}

TEST(RegisterClientPty, NoisePrefixIsSkipped) {
  Rig rig;
  rig.fake.set_mode(Mode::kNoisePrefix);
  RegisterClient client(rig.port, kId, kReplyTimeout);
  const Transaction t = ReadAngles(client);
  ExpectAngles(t);
  EXPECT_GT(t.noise_bytes_skipped, 0u);
}

TEST(RegisterClientPty, StaleFrameThenGoodFrame) {
  Rig rig;
  rig.fake.set_mode(Mode::kStaleThenGood);
  RegisterClient client(rig.port, kId, kReplyTimeout);
  const Transaction t = ReadAngles(client);
  ExpectAngles(t);
  EXPECT_EQ(t.frames_rejected, 1u);
  EXPECT_EQ(t.last_rejection, rs485::DecodeStatus::kWrongAddress);
}

TEST(RegisterClientPty, StaleInputBeforeTheRequestIsDiscarded) {
  Rig rig;
  RegisterClient client(rig.port, kId, kReplyTimeout);
  // A complete, valid-looking reply that nobody asked for (e.g. a late reply).
  rig.fake.inject(FakeRh56f1::reply(kId, 0x11, protocol::kRegAngleAct,
                                    std::vector<std::uint8_t>(12, 0x77)));
  std::this_thread::sleep_for(milliseconds(30));
  const Transaction t = ReadAngles(client);
  ExpectAngles(t);  // the injected 0x7777 values were not used
  EXPECT_EQ(t.stale_input_discarded, 20u);
}

TEST(RegisterClientPty, TwoRepliesInOneBurstDoNotLeakIntoTheNextRequest) {
  Rig rig;
  rig.fake.set_mode(Mode::kTwoReplies);
  RegisterClient client(rig.port, kId, kReplyTimeout);
  ExpectAngles(ReadAngles(client));
  std::this_thread::sleep_for(milliseconds(20));
  rig.fake.set_mode(Mode::kNormal);
  const Transaction second = ReadAngles(client);
  ExpectAngles(second);
  // The unsolicited frame is either dropped with the first transaction's
  // buffer or discarded before the second request; it is never parsed as the
  // second reply.
  EXPECT_EQ(second.frames_rejected, 0u);
}

struct RejectCase {
  Mode mode;
  ClientStatus status;
  bool expect_rejection;
  rs485::DecodeStatus rejection;
};

class RegisterClientPtyRejects : public ::testing::TestWithParam<RejectCase> {};

TEST_P(RegisterClientPtyRejects, InvalidReplyNeverBecomesData) {
  const RejectCase c = GetParam();
  Rig rig;
  rig.fake.set_mode(c.mode);
  RegisterClient client(rig.port, kId, milliseconds(80));
  const Transaction t = ReadAngles(client);
  EXPECT_EQ(t.status, c.status) << to_string(t.status);
  EXPECT_TRUE(t.payload.empty());
  if (c.expect_rejection) {
    EXPECT_GE(t.frames_rejected, 1u);
    EXPECT_EQ(t.last_rejection, c.rejection);
  }
}

INSTANTIATE_TEST_SUITE_P(
    Modes, RegisterClientPtyRejects,
    ::testing::Values(
        RejectCase{Mode::kTruncated, ClientStatus::kTimeout, false, rs485::DecodeStatus::kOk},
        RejectCase{Mode::kBadHeader, ClientStatus::kTimeout, false, rs485::DecodeStatus::kOk},
        RejectCase{Mode::kBadLength, ClientStatus::kTimeout, true,
                   rs485::DecodeStatus::kBadLength},
        RejectCase{Mode::kBadChecksum, ClientStatus::kTimeout, true,
                   rs485::DecodeStatus::kBadChecksum},
        RejectCase{Mode::kWrongId, ClientStatus::kTimeout, true,
                   rs485::DecodeStatus::kWrongHandId},
        RejectCase{Mode::kWrongCommand, ClientStatus::kTimeout, true,
                   rs485::DecodeStatus::kWrongCommand},
        RejectCase{Mode::kWrongAddress, ClientStatus::kTimeout, true,
                   rs485::DecodeStatus::kWrongAddress},
        RejectCase{Mode::kSilent, ClientStatus::kTimeout, false, rs485::DecodeStatus::kOk}));

TEST(RegisterClientPty, PeerDisconnectIsReportedAndNotRetried) {
  Rig rig;
  rig.fake.set_mode(Mode::kHangUpOnRequest);
  RegisterClient client(rig.port, kId, milliseconds(300));
  const Transaction t = ReadAngles(client);
  EXPECT_EQ(t.status, ClientStatus::kDisconnected) << to_string(t.status);
  EXPECT_EQ(client.read_requests_sent(), 1u);
}

TEST(RegisterClientPty, WritesAreDisabledByDefaultAndSendNothing) {
  Rig rig;
  RegisterClient client(rig.port, kId, kReplyTimeout);
  const Transaction t = client.write(protocol::kRegAngleSet, {1000, 1000, 1000, 1000, 1000, 1000});
  EXPECT_EQ(t.status, ClientStatus::kWritesDisabled);
  EXPECT_TRUE(t.request.empty());
  std::this_thread::sleep_for(milliseconds(50));
  EXPECT_EQ(rig.fake.bytes_received(), 0u);
  EXPECT_EQ(client.write_requests_sent(), 0u);
}

// The ACK/NAK paths use a register that cannot move the hand (FORCE_SET is a
// threshold); the fake has no actuators either way.
TEST(RegisterClientPty, WriteAckAndNakWhenExplicitlyEnabledInATest) {
  Rig rig;
  RegisterClient client(rig.port, kId, kReplyTimeout);
  client.enable_writes(true);
  const std::vector<std::int16_t> values = {100, 100, 100, 100, 100, 100};
  Transaction ack = client.write(protocol::kRegForceSet, values);
  EXPECT_EQ(ack.status, ClientStatus::kOk) << to_string(ack.status);
  EXPECT_THAT(ack.payload, ElementsAre(rs485::kWriteAcknowledged));
  rig.fake.set_mode(Mode::kNak);
  Transaction nak = client.write(protocol::kRegForceSet, values);
  EXPECT_EQ(nak.status, ClientStatus::kWriteNotAcknowledged) << to_string(nak.status);
  EXPECT_EQ(rig.fake.write_requests(), 2u);
}

// ---- two hands ------------------------------------------------------------------

TEST(RegisterClientPty, LeftAndRightAreIndependentAndOneTimeoutDoesNotBlockTheOther) {
  // Same bus ID on both sides: separate adapters, so no conflict.
  Rig left(1);
  Rig right(1);
  ASSERT_EQ(left.status, SerialStatus::kOk);
  ASSERT_EQ(right.status, SerialStatus::kOk);
  left.fake.set_mode(Mode::kSilent);
  RegisterClient left_client(left.port, 1, milliseconds(500));
  RegisterClient right_client(right.port, 1, milliseconds(500));

  auto left_result = std::async(std::launch::async, [&] { return ReadAngles(left_client); });
  const auto start = std::chrono::steady_clock::now();
  const Transaction r = ReadAngles(right_client);
  const auto right_elapsed = std::chrono::steady_clock::now() - start;
  ExpectAngles(r);
  EXPECT_LT(right_elapsed, milliseconds(250));
  EXPECT_EQ(left_result.get().status, ClientStatus::kTimeout);
  EXPECT_EQ(left.fake.read_requests(), 1u);
  EXPECT_EQ(right.fake.read_requests(), 1u);
}

TEST(RegisterClientPty, NoFdLeakAcrossManyTransactions) {
  FakeRh56f1 fake(kId);
  LoadAngles(fake);
  const std::size_t before = OpenFdCount();
  {
    SerialPort port;
    ASSERT_EQ(port.open(fake.slave_path(), kBaud), SerialStatus::kOk);
    RegisterClient client(port, kId, kReplyTimeout);
    for (int i = 0; i < 20; ++i) ExpectAngles(ReadAngles(client));
  }
  EXPECT_EQ(OpenFdCount(), before);
}

}  // namespace
}  // namespace rh56f1_hardware
