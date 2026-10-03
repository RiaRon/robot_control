// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// PTY-backed fake RH56F1 for tests (no real device). The code under test opens
// slave_path() like a tty; this fixture answers on the master side. Request
// parsing and reply building here are written independently of rs485_codec so
// the two check each other.

#include <poll.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rh56f1_hardware::testing {

class FakeRh56f1 {
 public:
  enum class Mode {
    kNormal,
    kFragmented,      // reply one byte at a time
    kNoisePrefix,     // garbage (with a false 0x90) before the reply
    kTruncated,       // only the first half of the reply
    kBadHeader,       // EB 90 instead of 90 EB
    kBadLength,       // LEN byte < 3
    kBadChecksum,
    kWrongId,
    kWrongCommand,
    kWrongAddress,
    kNak,             // write reply with ACK byte 0x00
    kSilent,
    kHangUpOnRequest, // close the master side when a request arrives
    kStaleThenGood,   // a valid reply for another address, then the real one
    kTwoReplies,      // the real reply plus an unsolicited one in one burst
  };

  explicit FakeRh56f1(std::uint8_t device_id) : device_id_(device_id) {
    char name[128] = {};
    if (::openpty(&master_, &slave_keepalive_, name, nullptr, nullptr) != 0) return;
    slave_path_ = name;
    termios tio{};
    ::tcgetattr(slave_keepalive_, &tio);
    ::cfmakeraw(&tio);  // no echo before the code under test configures it
    ::tcsetattr(slave_keepalive_, TCSANOW, &tio);
    running_ = true;
    thread_ = std::thread([this] { loop(); });
  }

  ~FakeRh56f1() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    close_master();
    if (slave_keepalive_ >= 0) ::close(slave_keepalive_);
  }

  FakeRh56f1(const FakeRh56f1&) = delete;
  FakeRh56f1& operator=(const FakeRh56f1&) = delete;

  bool ok() const { return !slave_path_.empty(); }
  const std::string& slave_path() const { return slave_path_; }
  void set_mode(Mode mode) { mode_ = mode; }
  void set_register(std::uint16_t address, std::int16_t value) {
    std::lock_guard<std::mutex> lock(mutex_);
    registers_[address] = value;
  }

  std::vector<std::vector<std::uint8_t>> requests() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }
  std::size_t bytes_received() const { return bytes_received_; }
  std::size_t write_requests() const { return write_requests_; }
  std::size_t read_requests() const { return read_requests_; }

  // Raw bytes to the code under test, outside any request (stale input).
  void inject(const std::vector<std::uint8_t>& bytes) { send(bytes); }

  static std::uint8_t sum(const std::vector<std::uint8_t>& frame) {
    unsigned total = 0;
    for (std::size_t i = 2; i < frame.size(); ++i) total += frame[i];
    return static_cast<std::uint8_t>(total & 0xFFu);
  }

  // A well-formed reply frame, built here without the codec.
  static std::vector<std::uint8_t> reply(std::uint8_t id, std::uint8_t cmd, std::uint16_t address,
                                         const std::vector<std::uint8_t>& data) {
    std::vector<std::uint8_t> f = {0x90, 0xEB, id, static_cast<std::uint8_t>(data.size() + 3),
                                   cmd, static_cast<std::uint8_t>(address & 0xFF),
                                   static_cast<std::uint8_t>(address >> 8)};
    f.insert(f.end(), data.begin(), data.end());
    f.push_back(sum(f));
    return f;
  }

 private:
  void close_master() {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (master_ >= 0) ::close(master_);
    master_ = -1;
  }

  void send(const std::vector<std::uint8_t>& bytes) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    if (master_ < 0) return;
    std::size_t off = 0;
    while (off < bytes.size()) {
      const ssize_t n = ::write(master_, bytes.data() + off, bytes.size() - off);
      if (n <= 0) return;
      off += static_cast<std::size_t>(n);
    }
  }

  std::vector<std::uint8_t> register_bytes(std::uint16_t address, std::size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::uint8_t> out;
    for (std::size_t i = 0; i < count / 2; ++i) {
      const auto it = registers_.find(static_cast<std::uint16_t>(address + i));
      const auto v = static_cast<std::uint16_t>(it == registers_.end() ? 0 : it->second);
      out.push_back(static_cast<std::uint8_t>(v & 0xFF));
      out.push_back(static_cast<std::uint8_t>(v >> 8));
    }
    return out;
  }

  void handle(const std::vector<std::uint8_t>& req) {
    const std::uint8_t id = req[2];
    const std::uint8_t cmd = req[4];
    const auto address = static_cast<std::uint16_t>(req[5] | (req[6] << 8));
    if (cmd == 0x12) ++write_requests_; else ++read_requests_;
    if (id != device_id_) return;  // a real bus member ignores other IDs

    std::vector<std::uint8_t> data =
        cmd == 0x11 ? register_bytes(address, req[7]) : std::vector<std::uint8_t>{0x01};
    std::vector<std::uint8_t> r = reply(id, cmd, address, data);
    switch (mode_.load()) {
      case Mode::kNormal:
        send(r);
        break;
      case Mode::kFragmented:
        for (const std::uint8_t b : r) {
          send({b});
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        break;
      case Mode::kNoisePrefix: {
        std::vector<std::uint8_t> burst = {0x00, 0x90, 0x13, 0xEB, 0x55};
        burst.insert(burst.end(), r.begin(), r.end());
        send(burst);
        break;
      }
      case Mode::kTruncated:
        send(std::vector<std::uint8_t>(r.begin(), r.begin() + static_cast<long>(r.size() / 2)));
        break;
      case Mode::kBadHeader:
        r[0] = 0xEB;
        r[1] = 0x90;
        send(r);
        break;
      case Mode::kBadLength: {
        std::vector<std::uint8_t> f = {0x90, 0xEB, id, 0x02, cmd, r[5]};
        f.push_back(sum(f));
        send(f);
        break;
      }
      case Mode::kBadChecksum:
        r.back() ^= 0x5A;
        send(r);
        break;
      case Mode::kWrongId:
        send(reply(static_cast<std::uint8_t>(id + 1), cmd, address, data));
        break;
      case Mode::kWrongCommand:
        send(reply(id, cmd == 0x11 ? 0x12 : 0x11, address, {0x01}));
        break;
      case Mode::kWrongAddress:
        send(reply(id, cmd, static_cast<std::uint16_t>(address + 6), data));
        break;
      case Mode::kNak:
        send(reply(id, cmd, address, cmd == 0x12 ? std::vector<std::uint8_t>{0x00} : data));
        break;
      case Mode::kSilent:
        break;
      case Mode::kHangUpOnRequest:
        close_master();
        break;
      case Mode::kStaleThenGood: {
        std::vector<std::uint8_t> burst =
            reply(id, cmd, static_cast<std::uint16_t>(address - 6), data);
        burst.insert(burst.end(), r.begin(), r.end());
        send(burst);
        break;
      }
      case Mode::kTwoReplies: {
        std::vector<std::uint8_t> burst = r;
        const auto extra = reply(id, cmd, static_cast<std::uint16_t>(address + 6), data);
        burst.insert(burst.end(), extra.begin(), extra.end());
        send(burst);
        break;
      }
    }
  }

  void loop() {
    std::vector<std::uint8_t> buffer;
    while (running_) {
      int fd;
      {
        std::lock_guard<std::mutex> lock(io_mutex_);
        fd = master_;
      }
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        continue;
      }
      pollfd pfd{fd, POLLIN, 0};
      if (::poll(&pfd, 1, 10) <= 0 || !(pfd.revents & POLLIN)) continue;
      std::uint8_t chunk[256];
      const ssize_t n = ::read(fd, chunk, sizeof(chunk));
      if (n <= 0) continue;
      bytes_received_ += static_cast<std::size_t>(n);
      buffer.insert(buffer.end(), chunk, chunk + n);
      // Extract complete EB 90 request frames.
      while (buffer.size() >= 4) {
        if (buffer[0] != 0xEB || buffer[1] != 0x90) {
          buffer.erase(buffer.begin());
          continue;
        }
        const std::size_t total = static_cast<std::size_t>(buffer[3]) + 5;
        if (buffer.size() < total) break;
        std::vector<std::uint8_t> req(buffer.begin(), buffer.begin() + static_cast<long>(total));
        buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(total));
        std::vector<std::uint8_t> body(req.begin(), req.end() - 1);
        if (sum(body) != req.back()) continue;  // a real device drops bad frames
        {
          std::lock_guard<std::mutex> lock(mutex_);
          requests_.push_back(req);
        }
        handle(req);
      }
    }
  }

  std::uint8_t device_id_;
  int master_ = -1;
  int slave_keepalive_ = -1;
  std::string slave_path_;
  std::atomic<bool> running_{false};
  std::atomic<Mode> mode_{Mode::kNormal};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::mutex io_mutex_;
  std::map<std::uint16_t, std::int16_t> registers_;
  std::vector<std::vector<std::uint8_t>> requests_;
  std::atomic<std::size_t> bytes_received_{0};
  std::atomic<std::size_t> write_requests_{0};
  std::atomic<std::size_t> read_requests_{0};
};

}  // namespace rh56f1_hardware::testing
