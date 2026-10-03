cmake_minimum_required(VERSION 3.22)
project(rh56f1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

set(CORE_SOURCES
  src/transport.cpp
  src/mock_transport.cpp
  src/rs485_transport.cpp
  src/command_policy.cpp
  src/backend.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${CORE_SOURCES}
  src/rh56f1_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Threads::Threads)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface rh56f1_hardware.xml)

# Stage 5B-2a/b: RS-485 codec, serial I/O and register client, plus the
# read-only rh56f1_diag tool. Deliberately NOT linked into the ros2_control
# plugin above: the plugin's Rs485Transport stays a fail-closed stub until the
# real-transport gate passes (docs/rh56f1-protocol-evidence.md).
set(RS485_SOURCES
  src/protocol.cpp
  src/rs485_codec.cpp
  src/serial_port.cpp
  src/register_client.cpp
)
add_library(rh56f1_rs485 STATIC ${RS485_SOURCES} src/diag.cpp)
set_target_properties(rh56f1_rs485 PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(rh56f1_rs485 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(rh56f1_diag src/rh56f1_diag_main.cpp)
target_link_libraries(rh56f1_diag rh56f1_rs485)
install(TARGETS rh56f1_diag DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # ROS-free: transport, policy and backend are exercised directly, with no
  # device and no controller_manager.
  ament_add_gmock(test_mock_transport test/test_mock_transport.cpp ${CORE_SOURCES})
  target_include_directories(test_mock_transport PRIVATE include)

  ament_add_gmock(test_command_policy test/test_command_policy.cpp ${CORE_SOURCES})
  target_include_directories(test_command_policy PRIVATE include)

  ament_add_gmock(test_backend test/test_backend.cpp ${CORE_SOURCES})
  target_include_directories(test_backend PRIVATE include)
  target_link_libraries(test_backend Threads::Threads)

  ament_add_gmock(test_rs485_stub test/test_rs485_stub.cpp ${CORE_SOURCES})
  target_include_directories(test_rs485_stub PRIVATE include)

  # Stage 5B-1 offline RS-485 codec.
  ament_add_gmock(test_rs485_codec test/test_rs485_codec.cpp)
  target_link_libraries(test_rs485_codec rh56f1_rs485)

  # Stage 5B-2a/b: serial I/O, register client and diag tool against a PTY
  # fake (test/fake_rh56f1.hpp). No real device path is used.
  ament_add_gmock(test_serial_transport test/test_serial_transport.cpp)
  target_link_libraries(test_serial_transport rh56f1_rs485 util Threads::Threads)

  ament_add_gmock(test_diag test/test_diag.cpp)
  target_link_libraries(test_diag rh56f1_rs485 util Threads::Threads)
endif()

ament_package()
