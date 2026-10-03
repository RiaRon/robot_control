// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/rs485_codec.hpp"

#include <utility>

namespace rh56f1_hardware {

namespace rs485 {
namespace {

bool valid_hand_id(std::uint8_t id) {
  return id >= protocol::kMinHandId && id <= protocol::kMaxHandId;
}

void begin_request(std::uint8_t hand_id, std::uint8_t length, std::uint8_t command,
                   std::uint16_t address, std::vector<std::uint8_t>& frame) {
  frame = {kRequestHeader0,
           kRequestHeader1,
           hand_id,
           length,
           command,
           static_cast<std::uint8_t>(address & 0xFF),
           static_cast<std::uint8_t>((address >> 8) & 0xFF)};
}

void finish(std::vector<std::uint8_t>& frame) {
  frame.push_back(checksum(frame.data() + 2, frame.size() - 2));
}

}  // namespace

std::uint8_t checksum(const std::uint8_t* bytes, std::size_t count) {
  std::uint8_t sum = 0;
  for (std::size_t i = 0; i < count; ++i) sum = static_cast<std::uint8_t>(sum + bytes[i]);
  return sum;
}

bool encode_read_request(std::uint8_t hand_id, std::uint16_t address, std::uint8_t byte_count,
                         std::vector<std::uint8_t>& out) {
  if (!valid_hand_id(hand_id) || byte_count == 0) return false;
  std::vector<std::uint8_t> frame;
  begin_request(hand_id, kCommandAndAddressBytes + 1, kCommandRead, address, frame);
  frame.push_back(byte_count);
  finish(frame);
  out = std::move(frame);
  return true;
}

bool encode_write_request(std::uint8_t hand_id, std::uint16_t address,
                          const std::vector<std::int16_t>& values, std::vector<std::uint8_t>& out) {
  const std::size_t data_bytes = values.size() * protocol::kBytesPerRegister;
  if (!valid_hand_id(hand_id) || values.empty() || data_bytes > kMaxDataBytes) return false;
  std::vector<std::uint8_t> frame;
  begin_request(hand_id, static_cast<std::uint8_t>(data_bytes + kCommandAndAddressBytes),
                kCommandWrite, address, frame);
  for (const std::int16_t value : values) {
    const auto bits = static_cast<std::uint16_t>(value);
    frame.push_back(static_cast<std::uint8_t>(bits & 0xFF));
    frame.push_back(static_cast<std::uint8_t>((bits >> 8) & 0xFF));
  }
  finish(frame);
  out = std::move(frame);
  return true;
}

const char* to_string(DecodeStatus status) {
  switch (status) {
    case DecodeStatus::kOk:
      return "ok";
    case DecodeStatus::kNeedMoreData:
      return "need_more_data";
    case DecodeStatus::kBadHeader:
      return "bad_header";
    case DecodeStatus::kBadLength:
      return "bad_length";
    case DecodeStatus::kBadChecksum:
      return "bad_checksum";
    case DecodeStatus::kWrongHandId:
      return "wrong_hand_id";
    case DecodeStatus::kWrongCommand:
      return "wrong_command";
    case DecodeStatus::kWrongAddress:
      return "wrong_address";
    case DecodeStatus::kWrongPayloadLength:
      return "wrong_payload_length";
    case DecodeStatus::kWriteNotAcknowledged:
      return "write_not_acknowledged";
  }
  return "unknown";
}

DecodeStatus decode_response(const std::uint8_t* bytes, std::size_t count,
                             const ExpectedResponse& expected, Response& out,
                             std::size_t* consumed) {
  if (consumed) *consumed = 0;
  if (count >= 1 && bytes[0] != kResponseHeader0) return DecodeStatus::kBadHeader;
  if (count >= 2 && bytes[1] != kResponseHeader1) return DecodeStatus::kBadHeader;
  if (count < 4) return DecodeStatus::kNeedMoreData;

  const std::size_t length = bytes[3];
  if (length < kCommandAndAddressBytes) return DecodeStatus::kBadLength;
  // A write acknowledgement's LEN is fixed; checking it before waiting for
  // the rest avoids stalling on a corrupted LEN byte.
  if (expected.command == kCommandWrite && length != kWriteAckFrameBytes - kFrameOverhead) {
    return DecodeStatus::kBadLength;
  }
  const std::size_t frame_bytes = length + kFrameOverhead;
  if (count < frame_bytes) return DecodeStatus::kNeedMoreData;

  if (checksum(bytes + 2, frame_bytes - 3) != bytes[frame_bytes - 1]) {
    return DecodeStatus::kBadChecksum;
  }
  if (bytes[2] != expected.hand_id) return DecodeStatus::kWrongHandId;
  if (bytes[4] != expected.command) return DecodeStatus::kWrongCommand;
  const auto address = static_cast<std::uint16_t>(bytes[5] | (bytes[6] << 8));
  if (address != expected.address) return DecodeStatus::kWrongAddress;

  const std::size_t payload_bytes = length - kCommandAndAddressBytes;
  if (expected.command == kCommandRead && payload_bytes != expected.payload_bytes) {
    return DecodeStatus::kWrongPayloadLength;
  }
  if (expected.command == kCommandWrite && bytes[7] != kWriteAcknowledged) {
    return DecodeStatus::kWriteNotAcknowledged;
  }

  out.hand_id = bytes[2];
  out.command = bytes[4];
  out.address = address;
  out.payload.assign(bytes + 7, bytes + 7 + payload_bytes);
  if (consumed) *consumed = frame_bytes;
  return DecodeStatus::kOk;
}

std::size_t find_response_header(const std::uint8_t* bytes, std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    if (bytes[i] != kResponseHeader0) continue;
    if (i + 1 == count || bytes[i + 1] == kResponseHeader1) return i;
  }
  return count;
}

bool decode_int16_le(const std::vector<std::uint8_t>& payload, std::vector<std::int16_t>& out) {
  if (payload.size() % protocol::kBytesPerRegister != 0) return false;
  std::vector<std::int16_t> values;
  values.reserve(payload.size() / protocol::kBytesPerRegister);
  for (std::size_t i = 0; i < payload.size(); i += protocol::kBytesPerRegister) {
    values.push_back(static_cast<std::int16_t>(payload[i] | (payload[i + 1] << 8)));
  }
  out = std::move(values);
  return true;
}

bool decode_dof_group(const std::vector<std::uint8_t>& payload,
                      std::array<std::int16_t, protocol::kDofCount>& out) {
  if (payload.size() != protocol::kDofGroupBytes) return false;
  std::vector<std::int16_t> values;
  if (!decode_int16_le(payload, values)) return false;
  for (std::size_t i = 0; i < protocol::kDofCount; ++i) out[i] = values[i];
  return true;
}

}  // namespace rs485
}  // namespace rh56f1_hardware
