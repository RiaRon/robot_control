// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

// Official RH56F1 register-level constants used by the RS-485 codec. The only
// place these numbers live. Source for every value:
// Inspire "RH56F1 series user manual", CN PRJ-02-TS-U-001 (V1.0.1 content) and
// EN PRJ-02-TS-U-015 ("RH56F1-User-ManualV1.2.pdf", 2026-09), section 2.5,
// tables 30-47; both editions agree on everything in this file. See
// robot_control/docs/rh56f1-protocol-evidence.md.
//
// Deliberately NOT here: raw angle ranges (the two manual editions and the
// vendor host-software screenshots disagree), any raw<->radian conversion, and
// any rate/timeout (none is published). Those are not protocol constants.

#include <cstddef>
#include <cstdint>
#include <string>

namespace rh56f1_hardware::protocol {

// Bus ID register range (manual 2.5.1): default 1, range 1-254.
inline constexpr std::uint8_t kMinHandId = 1;
inline constexpr std::uint8_t kMaxHandId = 254;

// Register word addresses (manual table 30). Each register is one INT16,
// little-endian; a six-DOF group is 12 bytes.
inline constexpr std::uint16_t kRegHandId = 1000;
inline constexpr std::uint16_t kRegClearError = 1003;
inline constexpr std::uint16_t kRegAngleSet = 1040;      // W/R, 0.1 deg, -1 = no motion
inline constexpr std::uint16_t kRegForceSet = 1046;      // W/R, g
inline constexpr std::uint16_t kRegSpeedSet = 1052;      // W/R, dimensionless
inline constexpr std::uint16_t kRegPositionAct = 1058;   // R, actuator stroke
inline constexpr std::uint16_t kRegAngleAct = 1064;      // R, 0.1 deg
inline constexpr std::uint16_t kRegForceAct = 1070;      // R, g
inline constexpr std::uint16_t kRegCurrentAct = 1076;    // R, mA
inline constexpr std::uint16_t kRegErrorCode = 1082;     // R, bit field below
inline constexpr std::uint16_t kRegStatus = 1088;        // R, status code
inline constexpr std::uint16_t kRegTemperature = 1094;   // R, deg C

inline constexpr std::size_t kDofCount = 6;
inline constexpr std::size_t kBytesPerRegister = 2;
inline constexpr std::size_t kDofGroupBytes = kDofCount * kBytesPerRegister;

// ANGLE_SET / POS_SET value meaning "this DOF does not move" (2.5.10, 2.5.11).
inline constexpr std::int16_t kSetValueNoMotion = -1;

// Vendor DOF order inside every six-register group (tables 31-48, EtherCAT
// PDO table 51). This is NOT the plugin's actuator order and says nothing
// about which URDF joint each DOF drives (see the evidence doc).
enum class VendorDof : std::size_t {
  kLittle = 0,
  kRing = 1,
  kMiddle = 2,
  kIndex = 3,
  kThumbBend = 4,
  kThumbRotation = 5,
};

const char* to_string(VendorDof dof);

// ERROR register bits (table 44).
inline constexpr std::uint16_t kErrorLockedRotor = 1u << 0;
inline constexpr std::uint16_t kErrorOverTemperature = 1u << 1;
inline constexpr std::uint16_t kErrorOverCurrent = 1u << 2;
inline constexpr std::uint16_t kErrorMotorAbnormal = 1u << 3;
inline constexpr std::uint16_t kErrorCommunication = 1u << 4;

// Names of the set ERROR bits, e.g. "locked_rotor|over_current"; "none" for 0;
// unknown bits are reported as "bitN".
std::string error_bits_to_string(std::uint16_t bits);

// STATUS register codes (table 46). Code 8 (E-stop) appears only in the CN
// V1.0.1 manual; code 4 is undefined in both editions.
const char* status_code_to_string(std::int16_t code);

}  // namespace rh56f1_hardware::protocol
