// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/protocol.hpp"

namespace rh56f1_hardware::protocol {

const char* to_string(VendorDof dof) {
  switch (dof) {
    case VendorDof::kLittle:
      return "little";
    case VendorDof::kRing:
      return "ring";
    case VendorDof::kMiddle:
      return "middle";
    case VendorDof::kIndex:
      return "index";
    case VendorDof::kThumbBend:
      return "thumb_bend";
    case VendorDof::kThumbRotation:
      return "thumb_rotation";
  }
  return "unknown";
}

std::string error_bits_to_string(std::uint16_t bits) {
  if (bits == 0) return "none";
  static constexpr const char* kNames[] = {"locked_rotor", "over_temperature", "over_current",
                                           "motor_abnormal", "communication"};
  std::string out;
  for (unsigned bit = 0; bit < 16; ++bit) {
    if ((bits & (1u << bit)) == 0) continue;
    if (!out.empty()) out += '|';
    out += bit < 5 ? std::string(kNames[bit]) : "bit" + std::to_string(bit);
  }
  return out;
}

const char* status_code_to_string(std::int16_t code) {
  switch (code) {
    case 0:
      return "releasing";
    case 1:
      return "gripping";
    case 2:
      return "position_reached";
    case 3:
      return "force_reached";
    case 5:
      return "current_protection_stop";
    case 6:
      return "locked_rotor_stop";
    case 7:
      return "fault_stop";
    case 8:
      return "emergency_stop";
    default:
      return "undefined";
  }
}

}  // namespace rh56f1_hardware::protocol
