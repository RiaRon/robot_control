// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Offline tests of the RH56F1 RS-485 codec. Three kinds of expectation, kept
// apart:
//  - OfficialGolden*: byte strings printed in the Inspire RH56F1 manual
//    (section 2.2 tables 6/7/11/12 and section 3 examples 6-9, identical in
//    the CN V1.0.1 and EN V1.2 files).
//  - ManualMisprint*: section 3 examples 1 and 5, whose printed checksum does
//    not match the documented rule; the test pins the rule, not the misprint.
//  - everything else: constructed frames whose checksum is computed here by
//    a separate helper, not taken from the codec under test.

#include <array>
#include <cstdint>
#include <vector>

#include "gmock/gmock.h"
#include "rh56f1_hardware/rs485_codec.hpp"

namespace rh56f1_hardware::rs485 {
namespace {

using Bytes = std::vector<std::uint8_t>;
using ::testing::ElementsAre;
using ::testing::ElementsAreArray;

// Independent reference: plain int sum, masked at the end.
std::uint8_t ReferenceChecksum(const Bytes& frame_without_checksum) {
  int sum = 0;
  for (std::size_t i = 2; i < frame_without_checksum.size(); ++i) sum += frame_without_checksum[i];
  return static_cast<std::uint8_t>(sum & 0xFF);
}

Bytes WithChecksum(Bytes frame) {
  frame.push_back(ReferenceChecksum(frame));
  return frame;
}

ExpectedResponse ReadOf(std::uint8_t id, std::uint16_t address, std::uint8_t bytes) {
  return ExpectedResponse{id, kCommandRead, address, bytes};
}

ExpectedResponse WriteOf(std::uint8_t id, std::uint16_t address) {
  return ExpectedResponse{id, kCommandWrite, address, 0};
}

DecodeStatus Decode(const Bytes& bytes, const ExpectedResponse& expected, Response* out = nullptr,
                    std::size_t* consumed = nullptr) {
  Response scratch;
  return decode_response(bytes.data(), bytes.size(), expected, out ? *out : scratch, consumed);
}

const std::vector<std::int16_t> kAngles1000x4_1400_1000 = {1000, 1000, 1000, 1000, 1400, 1000};
const std::vector<std::int16_t> kAll1000 = {1000, 1000, 1000, 1000, 1000, 1000};

// ---- official golden vectors ------------------------------------------------

TEST(Rs485Codec, OfficialGoldenReadAngleActRequest) {  // table 6, example 9
  Bytes frame;
  ASSERT_TRUE(encode_read_request(1, protocol::kRegAngleAct, protocol::kDofGroupBytes, frame));
  EXPECT_THAT(frame, ElementsAre(0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C, 0x4E));
}

TEST(Rs485Codec, OfficialGoldenReadAngleActResponse) {  // table 7
  const Bytes reply = {0x90, 0xEB, 0x01, 0x0F, 0x11, 0x28, 0x04, 0xE8, 0x03, 0xE8,
                       0x03, 0xE8, 0x03, 0xE8, 0x03, 0x78, 0x05, 0xE8, 0x03, 0x61};
  Response out;
  std::size_t consumed = 0;
  ASSERT_EQ(Decode(reply, ReadOf(1, protocol::kRegAngleAct, 12), &out, &consumed),
            DecodeStatus::kOk);
  EXPECT_EQ(consumed, reply.size());
  std::array<std::int16_t, protocol::kDofCount> dofs{};
  ASSERT_TRUE(decode_dof_group(out.payload, dofs));
  EXPECT_THAT(dofs, ElementsAreArray(kAngles1000x4_1400_1000));
}

TEST(Rs485Codec, OfficialGoldenWriteAngleSetRequest) {  // table 11
  Bytes frame;
  ASSERT_TRUE(encode_write_request(1, protocol::kRegAngleSet, kAngles1000x4_1400_1000, frame));
  EXPECT_THAT(frame, ElementsAre(0xEB, 0x90, 0x01, 0x0F, 0x12, 0x10, 0x04, 0xE8, 0x03, 0xE8,
                                 0x03, 0xE8, 0x03, 0xE8, 0x03, 0x78, 0x05, 0xE8, 0x03, 0x4A));
}

TEST(Rs485Codec, OfficialGoldenWriteAcknowledgement) {  // table 12, example 8
  const Bytes ack = {0x90, 0xEB, 0x01, 0x04, 0x12, 0x10, 0x04, 0x01, 0x2C};
  Response out;
  ASSERT_EQ(Decode(ack, WriteOf(1, protocol::kRegAngleSet), &out), DecodeStatus::kOk);
  EXPECT_THAT(out.payload, ElementsAre(kWriteAcknowledged));
}

TEST(Rs485Codec, OfficialGoldenSection3Examples6To8) {
  Bytes frame;
  ASSERT_TRUE(encode_write_request(1, protocol::kRegForceSet, kAll1000, frame));
  EXPECT_EQ(frame.back(), 0xBE);  // example 6
  ASSERT_TRUE(encode_write_request(1, protocol::kRegSpeedSet, kAll1000, frame));
  EXPECT_EQ(frame.back(), 0xC4);  // example 7
  ASSERT_TRUE(encode_write_request(1, protocol::kRegAngleSet, kAll1000, frame));
  EXPECT_EQ(frame.back(), 0xB8);  // example 8
  EXPECT_EQ(Decode({0x90, 0xEB, 0x01, 0x04, 0x12, 0x16, 0x04, 0x01, 0x32},
                   WriteOf(1, protocol::kRegForceSet)),
            DecodeStatus::kOk);
  EXPECT_EQ(Decode({0x90, 0xEB, 0x01, 0x04, 0x12, 0x1C, 0x04, 0x01, 0x38},
                   WriteOf(1, protocol::kRegSpeedSet)),
            DecodeStatus::kOk);
}

TEST(Rs485Codec, ManualMisprintExamples1And5FollowTheDocumentedRule) {
  // Example 1 prints EB 90 01 05 12 E8 03 02 00 06; the section 2.2.1 rule
  // gives 0x05 for those bytes.
  Bytes frame;
  ASSERT_TRUE(encode_write_request(1, protocol::kRegHandId, {2}, frame));
  EXPECT_EQ(frame.back(), 0x05);
  // Example 5 prints EB 90 01 05 12 F1 03 01 00 0B: its checksum 0x0B is
  // right for register 1007 (table 30, force sensor calibration, EF 03); the
  // printed address F1 03 (1009) is the inconsistent byte.
  constexpr std::uint16_t kForceSensorCalibration = 1007;  // table 30
  ASSERT_TRUE(encode_write_request(1, kForceSensorCalibration, {1}, frame));
  EXPECT_THAT(frame, ElementsAre(0xEB, 0x90, 0x01, 0x05, 0x12, 0xEF, 0x03, 0x01, 0x00, 0x0B));
  const Bytes printed = {0xEB, 0x90, 0x01, 0x05, 0x12, 0xF1, 0x03, 0x01, 0x00};
  EXPECT_NE(ReferenceChecksum(printed), 0x0B);
}

// ---- encode ----------------------------------------------------------------

TEST(Rs485Codec, EncodeChecksumMatchesIndependentReference) {
  Bytes frame;
  ASSERT_TRUE(encode_write_request(254, 0xABCD, {-1, 32767, -32768, 0, 1, 1740}, frame));
  const Bytes body(frame.begin(), frame.end() - 1);
  EXPECT_EQ(frame.back(), ReferenceChecksum(body));
}

TEST(Rs485Codec, EncodeIsLittleEndianIncludingNegativeValues) {
  Bytes frame;
  ASSERT_TRUE(encode_write_request(1, 0x1234, {protocol::kSetValueNoMotion, 0x0102}, frame));
  EXPECT_EQ(frame[5], 0x34);
  EXPECT_EQ(frame[6], 0x12);
  EXPECT_THAT(Bytes(frame.begin() + 7, frame.end() - 1), ElementsAre(0xFF, 0xFF, 0x02, 0x01));
}

TEST(Rs485Codec, EncodeRejectsInvalidIdsAndSizes) {
  Bytes frame = {0xAA};
  EXPECT_FALSE(encode_read_request(0, protocol::kRegAngleAct, 12, frame));
  EXPECT_FALSE(encode_read_request(255, protocol::kRegAngleAct, 12, frame));
  EXPECT_FALSE(encode_read_request(1, protocol::kRegAngleAct, 0, frame));
  EXPECT_FALSE(encode_write_request(1, protocol::kRegAngleSet, {}, frame));
  EXPECT_FALSE(encode_write_request(1, protocol::kRegAngleSet,
                                    std::vector<std::int16_t>(kMaxDataBytes / 2 + 1, 0), frame));
  EXPECT_THAT(frame, ElementsAre(0xAA));  // untouched on failure
  EXPECT_TRUE(encode_write_request(1, protocol::kRegAngleSet,
                                   std::vector<std::int16_t>(kMaxDataBytes / 2, 0), frame));
}

// ---- decode: rejection paths -----------------------------------------------

const Bytes kGoodReadReply = WithChecksum(
    {0x90, 0xEB, 0x02, 0x0F, 0x11, 0x28, 0x04, 0x84, 0x03, 0xCC, 0x06, 0x4C,
     0x04, 0xC8, 0x05, 0x70, 0x05, 0x58, 0x02});  // ID 2: 900, 1740, 1100, 1480, 1392, 600

TEST(Rs485Codec, DecodesConstructedReplyForAnotherId) {
  Response out;
  ASSERT_EQ(Decode(kGoodReadReply, ReadOf(2, protocol::kRegAngleAct, 12), &out),
            DecodeStatus::kOk);
  std::array<std::int16_t, protocol::kDofCount> dofs{};
  ASSERT_TRUE(decode_dof_group(out.payload, dofs));
  EXPECT_THAT(dofs, ElementsAre(900, 1740, 1100, 1480, 1392, 600));
  EXPECT_EQ(dofs[static_cast<std::size_t>(protocol::VendorDof::kThumbRotation)], 600);
}

TEST(Rs485Codec, EveryProperPrefixNeedsMoreData) {
  for (std::size_t n = 0; n < kGoodReadReply.size(); ++n) {
    const Bytes prefix(kGoodReadReply.begin(), kGoodReadReply.begin() + n);
    std::size_t consumed = 99;
    EXPECT_EQ(Decode(prefix, ReadOf(2, protocol::kRegAngleAct, 12), nullptr, &consumed),
              DecodeStatus::kNeedMoreData)
        << "prefix length " << n;
    EXPECT_EQ(consumed, 0u);
  }
}

TEST(Rs485Codec, RejectsBadHeaderIncludingRequestHeader) {
  Bytes frame = kGoodReadReply;
  frame[0] = 0xEB;
  frame[1] = 0x90;
  EXPECT_EQ(Decode(frame, ReadOf(2, protocol::kRegAngleAct, 12)), DecodeStatus::kBadHeader);
  EXPECT_EQ(Decode({0x90, 0x00}, ReadOf(2, protocol::kRegAngleAct, 12)), DecodeStatus::kBadHeader);
}

TEST(Rs485Codec, RejectsEveryCorruptedByte) {
  for (std::size_t i = 2; i < kGoodReadReply.size(); ++i) {
    if (i == 3) continue;  // LEN changes the frame size; covered below
    Bytes frame = kGoodReadReply;
    frame[i] ^= 0x01;
    EXPECT_NE(Decode(frame, ReadOf(2, protocol::kRegAngleAct, 12)), DecodeStatus::kOk)
        << "byte " << i;
  }
  Bytes frame = kGoodReadReply;
  frame.back() ^= 0xFF;
  EXPECT_EQ(Decode(frame, ReadOf(2, protocol::kRegAngleAct, 12)), DecodeStatus::kBadChecksum);
}

TEST(Rs485Codec, RejectsWrongIdCommandAddressAndLength) {
  const auto expected = ReadOf(2, protocol::kRegAngleAct, 12);
  EXPECT_EQ(Decode(kGoodReadReply, ReadOf(1, protocol::kRegAngleAct, 12)),
            DecodeStatus::kWrongHandId);
  EXPECT_EQ(Decode(kGoodReadReply, ReadOf(2, protocol::kRegForceAct, 12)),
            DecodeStatus::kWrongAddress);
  EXPECT_EQ(Decode(kGoodReadReply, ReadOf(2, protocol::kRegAngleAct, 10)),
            DecodeStatus::kWrongPayloadLength);
  const Bytes write_ack = WithChecksum({0x90, 0xEB, 0x02, 0x04, 0x12, 0x28, 0x04, 0x01});
  EXPECT_EQ(Decode(write_ack, expected), DecodeStatus::kWrongCommand);
  const Bytes short_len = WithChecksum({0x90, 0xEB, 0x02, 0x02, 0x11, 0x28});
  EXPECT_EQ(Decode(short_len, expected), DecodeStatus::kBadLength);
}

TEST(Rs485Codec, RejectsWriteReplyWithWrongLengthOrNoAck) {
  const auto expected = WriteOf(1, protocol::kRegAngleSet);
  EXPECT_EQ(Decode(WithChecksum({0x90, 0xEB, 0x01, 0x04, 0x12, 0x10, 0x04, 0x00}), expected),
            DecodeStatus::kWriteNotAcknowledged);
  // The save-result frame (table 10) carries 0xFF on failure.
  EXPECT_EQ(Decode(WithChecksum({0x90, 0xEB, 0x01, 0x04, 0x12, 0x10, 0x04, 0xFF}), expected),
            DecodeStatus::kWriteNotAcknowledged);
  EXPECT_EQ(Decode({0x90, 0xEB, 0x01, 0x05}, expected), DecodeStatus::kBadLength);
  // Section 3 examples 2-4 print acknowledgements that echo address+1; the
  // codec does not accept them as an ack for the requested address.
  EXPECT_EQ(Decode({0x90, 0xEB, 0x01, 0x04, 0x12, 0xEA, 0x03, 0x01, 0x05},
                   WriteOf(1, protocol::kRegHandId + 1)),
            DecodeStatus::kWrongAddress);
}

// ---- stream handling ----------------------------------------------------------

TEST(Rs485Codec, FindsHeaderAfterGarbageAndAtTheEnd) {
  const Bytes garbage = {0x00, 0x90, 0x00, 0xEB, 0x90, 0xEB, 0x01};
  EXPECT_EQ(find_response_header(garbage.data(), garbage.size()), 4u);
  const Bytes trailing = {0x11, 0x22, 0x90};
  EXPECT_EQ(find_response_header(trailing.data(), trailing.size()), 2u);
  const Bytes none = {0x11, 0x22};
  EXPECT_EQ(find_response_header(none.data(), none.size()), 2u);
}

TEST(Rs485Codec, ResynchronisesPastAFalseHeaderInsideGarbage) {
  // A stray 90 EB whose "frame" fails the checksum, then the real reply.
  Bytes stream = {0x55, 0x90, 0xEB, 0x02, 0x04, 0x12, 0x00, 0x00, 0x00, 0x00};
  stream.insert(stream.end(), kGoodReadReply.begin(), kGoodReadReply.end());
  const auto expected = ReadOf(2, protocol::kRegAngleAct, 12);
  std::size_t offset = 0;
  int frames = 0;
  while (offset < stream.size()) {
    offset += find_response_header(stream.data() + offset, stream.size() - offset);
    if (offset >= stream.size()) break;
    Response out;
    std::size_t consumed = 0;
    const auto status = decode_response(stream.data() + offset, stream.size() - offset, expected,
                                        out, &consumed);
    if (status == DecodeStatus::kOk) {
      ++frames;
      offset += consumed;
    } else if (status == DecodeStatus::kNeedMoreData) {
      break;
    } else {
      offset += 1;
    }
  }
  EXPECT_EQ(frames, 1);
  EXPECT_EQ(offset, stream.size());
}

TEST(Rs485Codec, Int16DecodingBoundariesAndOddLength) {
  std::vector<std::int16_t> values;
  ASSERT_TRUE(decode_int16_le({0xFF, 0xFF, 0xFF, 0x7F, 0x00, 0x80, 0x00, 0x00}, values));
  EXPECT_THAT(values, ElementsAre(-1, 32767, -32768, 0));
  EXPECT_FALSE(decode_int16_le({0x01, 0x02, 0x03}, values));
  std::array<std::int16_t, protocol::kDofCount> dofs{};
  EXPECT_FALSE(decode_dof_group(Bytes(10, 0), dofs));
  EXPECT_FALSE(decode_dof_group(Bytes(14, 0), dofs));
}

TEST(Rs485Codec, VendorDofOrderNames) {
  EXPECT_STREQ(protocol::to_string(protocol::VendorDof::kLittle), "little");
  EXPECT_STREQ(protocol::to_string(protocol::VendorDof::kThumbBend), "thumb_bend");
  EXPECT_STREQ(protocol::to_string(protocol::VendorDof::kThumbRotation), "thumb_rotation");
  EXPECT_EQ(static_cast<std::size_t>(protocol::VendorDof::kThumbRotation), protocol::kDofCount - 1);
}

}  // namespace
}  // namespace rh56f1_hardware::rs485
