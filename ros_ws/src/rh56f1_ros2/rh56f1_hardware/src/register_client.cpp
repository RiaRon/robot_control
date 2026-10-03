// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/register_client.hpp"

#include <utility>

namespace rh56f1_hardware {
namespace {

using Clock = std::chrono::steady_clock;

// Statuses decode_response returns only after the checksum matched, i.e. for
// a well-formed frame that belongs to some other request.
bool is_foreign_frame(rs485::DecodeStatus status) {
  return status == rs485::DecodeStatus::kWrongHandId ||
         status == rs485::DecodeStatus::kWrongCommand ||
         status == rs485::DecodeStatus::kWrongAddress ||
         status == rs485::DecodeStatus::kWrongPayloadLength;
}

ClientStatus from_serial(SerialStatus status) {
  switch (status) {
    case SerialStatus::kTimeout:
      return ClientStatus::kTimeout;
    case SerialStatus::kDisconnected:
      return ClientStatus::kDisconnected;
    case SerialStatus::kNotOpen:
      return ClientStatus::kNotOpen;
    default:
      return ClientStatus::kIoError;
  }
}

}  // namespace

const char* to_string(ClientStatus status) {
  switch (status) {
    case ClientStatus::kOk:
      return "ok";
    case ClientStatus::kInvalidRequest:
      return "invalid_request";
    case ClientStatus::kWritesDisabled:
      return "writes_disabled";
    case ClientStatus::kNotOpen:
      return "not_open";
    case ClientStatus::kTimeout:
      return "timeout";
    case ClientStatus::kDisconnected:
      return "disconnected";
    case ClientStatus::kIoError:
      return "io_error";
    case ClientStatus::kWriteNotAcknowledged:
      return "write_not_acknowledged";
  }
  return "unknown";
}

RegisterClient::RegisterClient(ByteStream& stream, std::uint8_t device_id,
                               std::chrono::milliseconds reply_timeout)
    : stream_(stream), device_id_(device_id), reply_timeout_(reply_timeout) {}

Transaction RegisterClient::read(std::uint16_t address, std::uint8_t byte_count) {
  std::vector<std::uint8_t> request;
  if (!rs485::encode_read_request(device_id_, address, byte_count, request)) {
    return Transaction{};
  }
  return exchange(std::move(request),
                  rs485::ExpectedResponse{device_id_, rs485::kCommandRead, address, byte_count});
}

Transaction RegisterClient::write(std::uint16_t address, const std::vector<std::int16_t>& values) {
  if (!writes_enabled_) {
    Transaction refused;
    refused.status = ClientStatus::kWritesDisabled;
    return refused;
  }
  std::vector<std::uint8_t> request;
  if (!rs485::encode_write_request(device_id_, address, values, request)) {
    return Transaction{};
  }
  return exchange(std::move(request),
                  rs485::ExpectedResponse{device_id_, rs485::kCommandWrite, address, 0});
}

Transaction RegisterClient::exchange(std::vector<std::uint8_t> request,
                                     const rs485::ExpectedResponse& expected) {
  Transaction t;
  if (!stream_.is_open()) {
    t.status = ClientStatus::kNotOpen;
    return t;
  }
  t.stale_input_discarded = stream_.discard_input();
  t.sent_at = Clock::now();
  const auto deadline = t.sent_at + reply_timeout_;
  const SerialStatus sent = stream_.write_all(request, reply_timeout_);
  if (expected.command == rs485::kCommandWrite) {
    ++write_requests_sent_;
  } else {
    ++read_requests_sent_;
  }
  t.request = std::move(request);
  if (sent != SerialStatus::kOk) {
    t.status = from_serial(sent);
    t.completed_at = Clock::now();
    return t;
  }

  std::vector<std::uint8_t> buffer;
  while (true) {
    // Consume everything decodable in the buffer.
    while (!buffer.empty()) {
      const std::size_t header = rs485::find_response_header(buffer.data(), buffer.size());
      t.noise_bytes_skipped += header;
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(header));
      if (buffer.empty()) break;

      rs485::Response reply;
      std::size_t consumed = 0;
      const auto status =
          rs485::decode_response(buffer.data(), buffer.size(), expected, reply, &consumed);
      if (status == rs485::DecodeStatus::kOk) {
        t.response.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(consumed));
        t.payload = std::move(reply.payload);
        t.status = ClientStatus::kOk;
        t.completed_at = Clock::now();
        return t;
      }
      if (status == rs485::DecodeStatus::kNeedMoreData) break;
      if (status == rs485::DecodeStatus::kWriteNotAcknowledged) {
        const std::size_t frame = buffer[3] + rs485::kFrameOverhead;
        t.response.assign(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(frame));
        t.status = ClientStatus::kWriteNotAcknowledged;
        t.completed_at = Clock::now();
        return t;
      }
      ++t.frames_rejected;
      t.last_rejection = status;
      // A checksum-valid frame for another request is dropped whole; anything
      // else is dropped one byte at a time to resynchronise.
      const std::size_t drop =
          is_foreign_frame(status) ? buffer[3] + rs485::kFrameOverhead : std::size_t{1};
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(drop));
    }

    const auto now = Clock::now();
    if (now >= deadline) {
      t.status = ClientStatus::kTimeout;
      t.completed_at = now;
      return t;
    }
    const SerialStatus got = stream_.read_some(
        buffer, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now));
    if (got != SerialStatus::kOk) {
      t.status = from_serial(got);
      t.completed_at = Clock::now();
      return t;
    }
  }
}

}  // namespace rh56f1_hardware
