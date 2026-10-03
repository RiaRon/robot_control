// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>

namespace rh56f1_hardware {
namespace {

using Clock = std::chrono::steady_clock;

bool to_speed(int baud, speed_t& speed) {
  switch (baud) {
    case 19200:
      speed = B19200;
      return true;
    case 57600:
      speed = B57600;
      return true;
    case 115200:
      speed = B115200;
      return true;
    case 921600:
      speed = B921600;
      return true;
    default:
      return false;
  }
}

int remaining_ms(Clock::time_point deadline) {
  const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
  return left.count() > 0 ? static_cast<int>(left.count()) : 0;
}

}  // namespace

const char* to_string(SerialStatus status) {
  switch (status) {
    case SerialStatus::kOk:
      return "ok";
    case SerialStatus::kNotOpen:
      return "not_open";
    case SerialStatus::kInvalidArgument:
      return "invalid_argument";
    case SerialStatus::kOpenFailed:
      return "open_failed";
    case SerialStatus::kPortBusy:
      return "port_busy";
    case SerialStatus::kConfigureFailed:
      return "configure_failed";
    case SerialStatus::kTimeout:
      return "timeout";
    case SerialStatus::kDisconnected:
      return "disconnected";
    case SerialStatus::kIoError:
      return "io_error";
  }
  return "unknown";
}

bool is_supported_rh56f1_baud(int baud) {
  speed_t unused;
  return to_speed(baud, unused);
}

SerialPort::~SerialPort() { close(); }

SerialStatus SerialPort::open(const std::string& path, int baud) {
  close();
  speed_t speed;
  if (path.empty() || !to_speed(baud, speed)) return SerialStatus::kInvalidArgument;

  const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    last_errno_ = errno;
    // EBUSY: another owner set TIOCEXCL on this tty.
    return last_errno_ == EBUSY ? SerialStatus::kPortBusy : SerialStatus::kOpenFailed;
  }
  // From here every failure closes fd before returning.
  if (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
    last_errno_ = errno;
    ::close(fd);
    return last_errno_ == EWOULDBLOCK ? SerialStatus::kPortBusy : SerialStatus::kOpenFailed;
  }
  if (::ioctl(fd, TIOCEXCL) != 0) {
    last_errno_ = errno;
    ::close(fd);
    return SerialStatus::kConfigureFailed;
  }
  termios tio{};
  if (::tcgetattr(fd, &tio) != 0) {
    last_errno_ = errno;
    ::close(fd);
    return SerialStatus::kConfigureFailed;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0 ||
      ::tcsetattr(fd, TCSANOW, &tio) != 0) {
    last_errno_ = errno;
    ::close(fd);
    return SerialStatus::kConfigureFailed;
  }
  ::tcflush(fd, TCIFLUSH);
  fd_ = fd;
  path_ = path;
  last_errno_ = 0;
  return SerialStatus::kOk;
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::ioctl(fd_, TIOCNXCL);
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
  }
  fd_ = -1;
}

SerialStatus SerialPort::write_all(const std::vector<std::uint8_t>& bytes,
                                   std::chrono::milliseconds timeout) {
  if (fd_ < 0) return SerialStatus::kNotOpen;
  const auto deadline = Clock::now() + timeout;
  std::size_t sent = 0;
  while (sent < bytes.size()) {
    const ssize_t n = ::write(fd_, bytes.data() + sent, bytes.size() - sent);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      last_errno_ = errno;
      return (errno == EIO || errno == ENXIO || errno == ENODEV) ? SerialStatus::kDisconnected
                                                                  : SerialStatus::kIoError;
    }
    pollfd pfd{fd_, POLLOUT, 0};
    const int ready = ::poll(&pfd, 1, remaining_ms(deadline));
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0) {
      last_errno_ = errno;
      return SerialStatus::kIoError;
    }
    if (ready == 0) return SerialStatus::kTimeout;
    if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) return SerialStatus::kDisconnected;
  }
  return SerialStatus::kOk;
}

SerialStatus SerialPort::read_some(std::vector<std::uint8_t>& out,
                                   std::chrono::milliseconds timeout) {
  if (fd_ < 0) return SerialStatus::kNotOpen;
  const auto deadline = Clock::now() + timeout;
  std::uint8_t buffer[256];
  // Poll first: in raw mode with VMIN = VTIME = 0, read() returns 0 when no
  // data is available, so 0 alone cannot mean end-of-file. A hang-up shows as
  // POLLHUP (and EIO from read on a PTY, 0 on a real tty after hang-up).
  while (true) {
    pollfd pfd{fd_, POLLIN, 0};
    const int ready = ::poll(&pfd, 1, remaining_ms(deadline));
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0) {
      last_errno_ = errno;
      return SerialStatus::kIoError;
    }
    if (ready == 0) return SerialStatus::kTimeout;
    if (pfd.revents & POLLIN) {
      const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
      if (n > 0) {
        out.insert(out.end(), buffer, buffer + n);
        return SerialStatus::kOk;
      }
      if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
      if (n < 0) {
        last_errno_ = errno;
        return (errno == EIO || errno == ENXIO || errno == ENODEV) ? SerialStatus::kDisconnected
                                                                    : SerialStatus::kIoError;
      }
      // Readable but nothing read: end-of-file after a hang-up.
      if (pfd.revents & POLLHUP) return SerialStatus::kDisconnected;
      if (Clock::now() >= deadline) return SerialStatus::kTimeout;
      continue;
    }
    if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL)) return SerialStatus::kDisconnected;
  }
}

std::size_t SerialPort::discard_input() {
  if (fd_ < 0) return 0;
  std::size_t dropped = 0;
  std::uint8_t buffer[256];
  while (true) {
    const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
    if (n > 0) {
      dropped += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) continue;
    break;
  }
  ::tcflush(fd_, TCIFLUSH);
  return dropped;
}

}  // namespace rh56f1_hardware
