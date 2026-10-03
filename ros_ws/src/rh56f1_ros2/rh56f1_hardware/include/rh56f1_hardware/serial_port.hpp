// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Byte-stream I/O for one RH56F1 RS-485 adapter (Stage 5B-2a). Blocking with
// explicit timeouts; never call it from the ros2_control update loop.
//
// Ownership: one SerialPort object owns one fd and must be used by one thread.
// open() takes an exclusive flock() and TIOCEXCL on the device, so a second
// owner (another thread, transport or process) fails with kPortBusy instead of
// interleaving frames. Opening never writes a byte.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rh56f1_hardware {

enum class SerialStatus {
  kOk,
  kNotOpen,
  kInvalidArgument,  // unsupported baud, empty path
  kOpenFailed,
  kPortBusy,         // another owner holds the device
  kConfigureFailed,  // termios setup refused
  kTimeout,
  kDisconnected,     // peer hung up / device removed (EIO, POLLHUP, EOF)
  kIoError,
};

const char* to_string(SerialStatus status);

// Baud rates the RH56F1 BAUD register can select over RS-485 (manual 2.5.2:
// 0 = 115200, 1 = 57600, 2 = 19200, 3 = 921600). Anything else is refused.
bool is_supported_rh56f1_baud(int baud);

// Abstract so the register client can be tested without a tty.
class ByteStream {
 public:
  virtual ~ByteStream() = default;
  virtual bool is_open() const = 0;
  // Writes every byte or fails; handles partial writes, EINTR and EAGAIN.
  virtual SerialStatus write_all(const std::vector<std::uint8_t>& bytes,
                                 std::chrono::milliseconds timeout) = 0;
  // Appends whatever arrives within `timeout` (at least one byte on kOk).
  virtual SerialStatus read_some(std::vector<std::uint8_t>& out,
                                 std::chrono::milliseconds timeout) = 0;
  // Drops pending input (kernel buffer and anything immediately readable).
  // Returns the number of bytes discarded.
  virtual std::size_t discard_input() = 0;
};

// POSIX tty, raw mode, 8 data bits, no parity, 1 stop bit, no flow control
// (manual 2.2: "8 data bits, 1 stop bit, no parity").
class SerialPort final : public ByteStream {
 public:
  SerialPort() = default;
  ~SerialPort() override;
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  SerialStatus open(const std::string& path, int baud);
  void close();

  bool is_open() const override { return fd_ >= 0; }
  SerialStatus write_all(const std::vector<std::uint8_t>& bytes,
                         std::chrono::milliseconds timeout) override;
  SerialStatus read_some(std::vector<std::uint8_t>& out,
                         std::chrono::milliseconds timeout) override;
  std::size_t discard_input() override;

  const std::string& path() const { return path_; }
  // errno of the last failure, for diagnostics.
  int last_errno() const { return last_errno_; }

 private:
  int fd_ = -1;
  std::string path_;
  int last_errno_ = 0;
};

}  // namespace rh56f1_hardware
