// Copyright 2026 KUKU Robot Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <iostream>
#include <string>
#include <vector>

#include "rh56f1_hardware/diag.hpp"

int main(int argc, char** argv) {
  using namespace rh56f1_hardware::diag;
  std::vector<std::string> args(argv + 1, argv + argc);
  for (const auto& a : args) {
    if (a == "-h" || a == "--help") {
      std::cout << usage();
      return kExitOk;
    }
  }
  DiagOptions options;
  std::string error;
  if (!parse_args(args, options, error)) {
    std::cerr << "rh56f1_diag: " << error << "\n" << usage();
    return kExitUsage;
  }
  return run(options, std::cout, default_port_opener());
}
