cmake_minimum_required(VERSION 3.16)
project(barq_hw)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Monorepo single source of truth: the SAME protocol codec + superloop logic the Teensy
# builds (barq_firmware is COLCON_IGNOREd; PlatformIO builds it for the MCU, we build it
# for the host). If you move directories, fix this path.
set(FIRMWARE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../barq_firmware/src)

add_library(${PROJECT_NAME} SHARED
  src/barq_system.cpp
  ${FIRMWARE_SRC}/protocol.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${FIRMWARE_SRC}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface barq_hw_plugin.xml)

# Host twin of the Teensy firmware (real LoopCore on a PTY) — `ros2 run barq_hw teensy_emulator`.
add_executable(teensy_emulator
  src/teensy_emulator.cpp
  ${FIRMWARE_SRC}/protocol.cpp
  ${FIRMWARE_SRC}/loop_core.cpp
)
target_include_directories(teensy_emulator PRIVATE ${FIRMWARE_SRC})
target_link_libraries(teensy_emulator util)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(TARGETS teensy_emulator DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS test/integration_pty.py DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
