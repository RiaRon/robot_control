// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// RH56F1 register client over one ByteStream (Stage 5B-2a): one request, one
// validated reply, built on the offline codec in rs485_codec.hpp.
//
// - Blocking; call it from an I/O thread, never from the ros2_control loop.
// - Not thread-safe: one client per port, one thread per client.
// - Before each request, pending input is discarded so a late reply to an
//   earlier request cannot be taken as this request's reply. While waiting,
//   complete frames with a valid checksum that do not match this request
//   (another ID, command, address or length) are counted and skipped; noise
//   and corrupted frames are skipped byte by byte until a 90 EB header lines up.
// - Writes are DISABLED by default and nothing in this package enables them
//   outside tests: write() returns kWritesDisabled and sends nothing. There is
//   no reconnect logic here; a caller that sees kDisconnected must decide.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "rh56f1_hardware/rs485_codec.hpp"
#include "rh56f1_hardware/serial_port.hpp"

namespace rh56f1_hardware {

enum class ClientStatus {
  kOk,
  kInvalidRequest,        // bad ID/length, rejected before any byte is sent
  kWritesDisabled,        // write() called without enable_writes(true)
  kNotOpen,
  kTimeout,               // no matching reply within the reply timeout
  kDisconnected,
  kIoError,
  kWriteNotAcknowledged,  // matching write reply whose ACK byte is not 0x01
};

const char* to_string(ClientStatus status);

struct Transaction {
  ClientStatus status = ClientStatus::kInvalidRequest;
  std::vector<std::uint8_t> request;   // bytes sent (empty if nothing was sent)
  std::vector<std::uint8_t> response;  // the accepted reply frame
  std::vector<std::uint8_t> payload;   // register bytes (read) or {ack} (write)
  std::size_t stale_input_discarded = 0;  // bytes dropped before sending
  std::size_t noise_bytes_skipped = 0;    // bytes skipped to find a header
  std::size_t frames_rejected = 0;        // complete frames that did not match
  rs485::DecodeStatus last_rejection = rs485::DecodeStatus::kOk;
  std::chrono::steady_clock::time_point sent_at{};
  std::chrono::steady_clock::time_point completed_at{};
};

class RegisterClient {
 public:
  RegisterClient(ByteStream& stream, std::uint8_t device_id,
                 std::chrono::milliseconds reply_timeout);

  std::uint8_t device_id() const { return device_id_; }

  Transaction read(std::uint16_t address, std::uint8_t byte_count);

  // Only for tests of the ACK/NAK path until the real-transport gate passes.
  void enable_writes(bool enabled) { writes_enabled_ = enabled; }
  bool writes_enabled() const { return writes_enabled_; }
  Transaction write(std::uint16_t address, const std::vector<std::int16_t>& values);

  std::uint64_t write_requests_sent() const { return write_requests_sent_; }
  std::uint64_t read_requests_sent() const { return read_requests_sent_; }

 private:
  Transaction exchange(std::vector<std::uint8_t> request,
                       const rs485::ExpectedResponse& expected);

  ByteStream& stream_;
  std::uint8_t device_id_;
  std::chrono::milliseconds reply_timeout_;
  bool writes_enabled_ = false;
  std::uint64_t write_requests_sent_ = 0;
  std::uint64_t read_requests_sent_ = 0;
};

}  // namespace rh56f1_hardware
