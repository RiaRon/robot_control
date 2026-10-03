// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/diag.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>

#include "rh56f1_hardware/register_client.hpp"
#include "rh56f1_hardware/rs485_codec.hpp"

namespace rh56f1_hardware::diag {
namespace {

bool parse_int(const std::string& text, int& out) {
  try {
    std::size_t used = 0;
    const long value = std::stol(text, &used, 10);
    if (used != text.size()) return false;
    out = static_cast<int>(value);
    return static_cast<long>(out) == value;
  } catch (...) {
    return false;
  }
}

bool parse_double(const std::string& text, double& out) {
  try {
    std::size_t used = 0;
    out = std::stod(text, &used);
    return used == text.size() && std::isfinite(out);
  } catch (...) {
    return false;
  }
}

std::string hex(const std::vector<std::uint8_t>& bytes) {
  std::ostringstream s;
  s << std::hex << std::uppercase << std::setfill('0');
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i) s << ' ';
    s << std::setw(2) << static_cast<int>(bytes[i]);
  }
  return s.str();
}

std::string wall_timestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()).count() % 1000;
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &tm);
  std::ostringstream s;
  s << buffer << '.' << std::setw(3) << std::setfill('0') << ms << 'Z';
  return s.str();
}

void print_values(std::ostream& out, const DiagRegister& reg,
                  const std::array<std::int16_t, protocol::kDofCount>& values) {
  for (std::size_t i = 0; i < protocol::kDofCount; ++i) {
    out << "    " << std::left << std::setw(15)
        << protocol::to_string(static_cast<protocol::VendorDof>(i)) << std::right << values[i];
    if (reg.address == protocol::kRegErrorCode) {
      out << "  [" << protocol::error_bits_to_string(static_cast<std::uint16_t>(values[i])) << "]";
    } else if (reg.address == protocol::kRegStatus) {
      out << "  [" << protocol::status_code_to_string(values[i]) << "]";
    }
    out << '\n';
  }
}

}  // namespace

const char* const kPowerOnWarning =
    "WARNING: this tool only sends read requests, but no software can guarantee that the\n"
    "hand does not move when it is powered on or when its serial link opens. The vendor\n"
    "does not document power-on behavior. Keep the hand clear of people and objects,\n"
    "keep its power switch within reach, and confirm the port belongs to the intended\n"
    "hand before running with --execute-read-only.\n";

std::string usage() {
  std::ostringstream s;
  s << "usage: rh56f1_diag --side left|right --port /dev/serial/by-id/<adapter>\n"
       "                   --device-id <1-254> --baud <115200|57600|19200|921600>\n"
       "                   [--dry-run | --execute-read-only]\n"
       "                   [--samples N --period-sec S] [--reply-timeout-ms MS]\n"
       "Dry run (default) prints the read frames and never opens the port.\n"
       "--execute-read-only opens the port and reads ANGLE_ACT, FORCE_ACT, CURRENT_ACT,\n"
       "ERROR, STATUS, TEMP once (raw vendor units). Nothing is ever written.\n"
       "--samples 2.."
    << kMaxSamples << " requires --period-sec " << kMinPeriodSec << ".." << kMaxPeriodSec
    << ". Reply timeout " << kMinReplyTimeoutMs << ".." << kMaxReplyTimeoutMs
    << " ms (default " << kDefaultReplyTimeoutMs << ").\n";
  return s.str();
}

bool parse_args(const std::vector<std::string>& args, DiagOptions& out, std::string& error) {
  DiagOptions o;
  bool dry_run = false;
  bool have_device_id = false;
  bool have_baud = false;
  bool have_period = false;
  bool have_samples = false;
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    auto value = [&](std::string& v) {
      if (i + 1 >= args.size()) {
        error = a + " needs a value";
        return false;
      }
      v = args[++i];
      return true;
    };
    std::string v;
    if (a == "--side") {
      if (!value(o.side)) return false;
    } else if (a == "--port") {
      if (!value(o.port)) return false;
    } else if (a == "--device-id") {
      if (!value(v)) return false;
      if (!parse_int(v, o.device_id)) return error = "--device-id must be an integer", false;
      have_device_id = true;
    } else if (a == "--baud") {
      if (!value(v)) return false;
      if (!parse_int(v, o.baud)) return error = "--baud must be an integer", false;
      have_baud = true;
    } else if (a == "--samples") {
      if (!value(v)) return false;
      if (!parse_int(v, o.samples)) return error = "--samples must be an integer", false;
      have_samples = true;
    } else if (a == "--period-sec") {
      if (!value(v)) return false;
      if (!parse_double(v, o.period_sec)) return error = "--period-sec must be a number", false;
      have_period = true;
    } else if (a == "--reply-timeout-ms") {
      if (!value(v)) return false;
      if (!parse_int(v, o.reply_timeout_ms)) {
        return error = "--reply-timeout-ms must be an integer", false;
      }
    } else if (a == "--dry-run") {
      dry_run = true;
    } else if (a == "--execute-read-only") {
      o.execute_read_only = true;
    } else if (a == "--allow-non-by-id-port") {
      o.allow_non_by_id_port = true;
    } else {
      error = "unknown option " + a;
      return false;
    }
  }
  if (o.side != "left" && o.side != "right") return error = "--side must be left or right", false;
  if (o.port.empty()) return error = "--port is required", false;
  if (!o.allow_non_by_id_port && o.port.rfind(kByIdPrefix, 0) != 0) {
    return error = std::string("--port must be a stable ") + kByIdPrefix + "... path", false;
  }
  if (!have_device_id) return error = "--device-id is required (no ID scan)", false;
  if (o.device_id < protocol::kMinHandId || o.device_id > protocol::kMaxHandId) {
    return error = "--device-id must be 1-254", false;
  }
  if (!have_baud) return error = "--baud is required", false;
  if (!is_supported_rh56f1_baud(o.baud)) {
    return error = "--baud must be 115200, 57600, 19200 or 921600", false;
  }
  if (dry_run && o.execute_read_only) {
    return error = "--dry-run and --execute-read-only are mutually exclusive", false;
  }
  if (o.samples < 1 || o.samples > kMaxSamples) {
    return error = "--samples must be 1-" + std::to_string(kMaxSamples), false;
  }
  if (have_samples && o.samples > 1 && !have_period) {
    return error = "--samples > 1 requires --period-sec", false;
  }
  if (have_period && (o.period_sec < kMinPeriodSec || o.period_sec > kMaxPeriodSec)) {
    std::ostringstream s;
    s << "--period-sec must be " << kMinPeriodSec << "-" << kMaxPeriodSec;
    return error = s.str(), false;
  }
  if (have_period && o.samples == 1) {
    return error = "--period-sec is only meaningful with --samples > 1", false;
  }
  if (o.reply_timeout_ms < kMinReplyTimeoutMs || o.reply_timeout_ms > kMaxReplyTimeoutMs) {
    return error = "--reply-timeout-ms out of range", false;
  }
  out = o;
  return true;
}

PortOpener default_port_opener() {
  return [](const DiagOptions& options, std::string& error) -> std::unique_ptr<ByteStream> {
    auto port = std::make_unique<SerialPort>();
    const SerialStatus status = port->open(options.port, options.baud);
    if (status != SerialStatus::kOk) {
      error = std::string(to_string(status)) + " (errno " + std::to_string(port->last_errno()) + ")";
      return nullptr;
    }
    return port;
  };
}

int run(const DiagOptions& options, std::ostream& out, const PortOpener& opener,
        const std::function<void(std::chrono::milliseconds)>& sleep) {
  const auto id = static_cast<std::uint8_t>(options.device_id);
  out << "rh56f1_diag side=" << options.side << " port=" << options.port
      << " device_id=" << options.device_id << " baud=" << options.baud << " 8N1"
      << " mode=" << (options.execute_read_only ? "execute-read-only" : "dry-run") << '\n';
  out << "units: vendor raw values only (no radian conversion)\n";

  if (!options.execute_read_only) {
    out << "DRY RUN: the port is NOT opened. Read requests that --execute-read-only would send:\n";
    for (const auto& reg : kDiagRegisters) {
      std::vector<std::uint8_t> frame;
      rs485::encode_read_request(id, reg.address, protocol::kDofGroupBytes, frame);
      out << "  " << std::left << std::setw(12) << reg.name << std::right << " addr " << reg.address
          << " : " << hex(frame) << '\n';
    }
    out << kPowerOnWarning;
    return kExitOk;
  }

  out << kPowerOnWarning;
  std::string error;
  std::unique_ptr<ByteStream> port = opener(options, error);
  if (!port) {
    out << "ERROR: could not open " << options.port << ": " << error << '\n';
    return kExitOpenFailed;
  }
  RegisterClient client(*port, id, std::chrono::milliseconds(options.reply_timeout_ms));

  bool all_ok = true;
  for (int sample = 0; sample < options.samples; ++sample) {
    if (sample > 0) {
      const auto period =
          std::chrono::milliseconds(static_cast<long long>(options.period_sec * 1000.0));
      if (sleep) sleep(period); else std::this_thread::sleep_for(period);
    }
    out << "sample " << (sample + 1) << "/" << options.samples << " at " << wall_timestamp()
        << '\n';
    for (const auto& reg : kDiagRegisters) {
      const Transaction t = client.read(reg.address, protocol::kDofGroupBytes);
      const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t.completed_at -
                                                                            t.sent_at).count();
      out << "  " << reg.name << " addr " << reg.address << " (" << reg.unit << ")\n"
          << "    request  : " << hex(t.request) << '\n'
          << "    response : " << (t.response.empty() ? "-" : hex(t.response)) << '\n'
          << "    status   : " << to_string(t.status) << "  round_trip_us=" << us
          << " stale_discarded=" << t.stale_input_discarded
          << " noise_skipped=" << t.noise_bytes_skipped << " frames_rejected="
          << t.frames_rejected;
      if (t.frames_rejected) out << " last_rejection=" << rs485::to_string(t.last_rejection);
      out << '\n';
      std::array<std::int16_t, protocol::kDofCount> values{};
      if (t.status == ClientStatus::kOk && rs485::decode_dof_group(t.payload, values)) {
        print_values(out, reg, values);
      } else {
        all_ok = false;
      }
      if (t.status == ClientStatus::kDisconnected) {
        out << "ERROR: port disconnected; stopping (no reconnect)\n";
        return kExitReadFailed;
      }
    }
  }
  out << "writes sent: " << client.write_requests_sent() << '\n';
  return all_ok ? kExitOk : kExitReadFailed;
}

}  // namespace rh56f1_hardware::diag
