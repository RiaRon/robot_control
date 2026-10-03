// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "rh56f1_hardware/transport.hpp"

namespace rh56f1_hardware {

const char* to_string(TransportError error) {
  switch (error) {
    case TransportError::kNone:
      return "none";
    case TransportError::kNotConnected:
      return "not_connected";
    case TransportError::kTimeout:
      return "timeout";
    case TransportError::kCommunicationError:
      return "communication_error";
    case TransportError::kInvalidResponse:
      return "invalid_response";
    case TransportError::kInvalidCommand:
      return "invalid_command";
    case TransportError::kNotImplemented:
      return "not_implemented";
  }
  return "unknown";
}

}  // namespace rh56f1_hardware
