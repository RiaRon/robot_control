// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Offline codec for the RH56F1 RS-485 register protocol (Stage 5B-1). Pure
// byte encode/decode: it never opens a port and is not linked into the
// ros2_control plugin. Frame layout, command codes and checksum: RH56F1 user
// manual section 2.2, tables 4-12 (identical in the CN V1.0.1 and EN V1.2
// files); see robot_control/docs/rh56f1-protocol-evidence.md.
//
//   request : EB 90 | ID | LEN | CMD | ADDR_L ADDR_H | DATA... | CS
//   response: 90 EB | ID | LEN | CMD | ADDR_L ADDR_H | DATA... | CS
//   LEN = bytes from CMD to the last DATA byte (read request: 4, the DATA
//   being the requested byte count); CS = low byte of the sum of ID..DATA.
//
// The manual defines no error/NAK frame. Any response that is not exactly the
// expected frame is rejected with a specific DecodeStatus.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "rh56f1_hardware/protocol.hpp"

namespace rh56f1_hardware::rs485 {

inline constexpr std::uint8_t kRequestHeader0 = 0xEB;
inline constexpr std::uint8_t kRequestHeader1 = 0x90;
inline constexpr std::uint8_t kResponseHeader0 = 0x90;
inline constexpr std::uint8_t kResponseHeader1 = 0xEB;
inline constexpr std::uint8_t kCommandRead = 0x11;
inline constexpr std::uint8_t kCommandWrite = 0x12;
// Byte 7 of a write acknowledgement (table 9).
inline constexpr std::uint8_t kWriteAcknowledged = 0x01;

// header(2) + ID + LEN + CS.
inline constexpr std::size_t kFrameOverhead = 5;
// CMD + ADDR_L + ADDR_H, counted in LEN.
inline constexpr std::size_t kCommandAndAddressBytes = 3;
// LEN is one byte.
inline constexpr std::size_t kMaxDataBytes = 0xFF - kCommandAndAddressBytes;
// A write acknowledgement is always 9 bytes (LEN = 4).
inline constexpr std::size_t kWriteAckFrameBytes = 9;

std::uint8_t checksum(const std::uint8_t* bytes, std::size_t count);

// Returns false (out untouched) for an ID outside 1-254 or a byte count of 0.
bool encode_read_request(std::uint8_t hand_id, std::uint16_t address, std::uint8_t byte_count,
                         std::vector<std::uint8_t>& out);

// Returns false (out untouched) for an ID outside 1-254, no values, or more
// values than fit in one frame. Values are sent as INT16 little-endian.
bool encode_write_request(std::uint8_t hand_id, std::uint16_t address,
                          const std::vector<std::int16_t>& values, std::vector<std::uint8_t>& out);

enum class DecodeStatus {
  kOk,
  kNeedMoreData,        // a valid prefix of a frame; wait for more bytes
  kBadHeader,
  kBadLength,           // LEN inconsistent with the command
  kBadChecksum,
  kWrongHandId,
  kWrongCommand,
  kWrongAddress,
  kWrongPayloadLength,  // read reply with a byte count other than requested
  kWriteNotAcknowledged,
};

const char* to_string(DecodeStatus status);

struct ExpectedResponse {
  std::uint8_t hand_id = 0;
  std::uint8_t command = 0;       // kCommandRead or kCommandWrite
  std::uint16_t address = 0;
  std::uint8_t payload_bytes = 0; // read only: requested byte count
};

struct Response {
  std::uint8_t hand_id = 0;
  std::uint8_t command = 0;
  std::uint16_t address = 0;
  std::vector<std::uint8_t> payload;  // read: register bytes; write: {ack}
};

// Decodes one frame that starts at bytes[0]. On kOk, *consumed is the frame
// length. On kNeedMoreData nothing is consumed. On any other status the
// caller should drop one byte and resynchronise (see find_response_header).
DecodeStatus decode_response(const std::uint8_t* bytes, std::size_t count,
                             const ExpectedResponse& expected, Response& out,
                             std::size_t* consumed);

// Offset of the first 0x90 0xEB pair, or of a trailing lone 0x90 (a header
// that may still be completing), or `count` if neither is present.
std::size_t find_response_header(const std::uint8_t* bytes, std::size_t count);

// INT16 little-endian register values; false for an odd byte count.
bool decode_int16_le(const std::vector<std::uint8_t>& payload, std::vector<std::int16_t>& out);

// One six-register group in vendor DOF order; false unless exactly 12 bytes.
bool decode_dof_group(const std::vector<std::uint8_t>& payload,
                      std::array<std::int16_t, protocol::kDofCount>& out);

}  // namespace rh56f1_hardware::rs485
