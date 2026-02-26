cmake_minimum_required(VERSION 3.16)
project(kafeiche_drivers)

# ==============================
# C++ Standard & Warnings
# ==============================
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(backward_ros REQUIRED)

# ==============================
# pigpio libraries (daemon interface only)
# ==============================
# We only link the pigpiod_if2 client library; the legacy pigpio library is
# intentionally omitted because all code now uses the daemon API.
find_library(PIGPIOD_LIBRARIES pigpiod_if2 REQUIRED)
if(NOT PIGPIOD_LIBRARIES)
  message(FATAL_ERROR "pigpiod_if2 library not found (install dev package?)")
endif()

# ==============================
# Include dirs
# ==============================
include_directories(include)

# Note: the servo_motor node is no longer required now that
# the hardware interface controls the motors directly.
# The source file is kept for reference but is not built.
# If desired, remove src/servo_motor.cpp from the repository.

# Add shared library for hardware interface plugin
add_library(
  controller
  SHARED
  src/controller.cpp
)
# link against the daemon API library only
if(PIGPIOD_LIBRARIES)
  target_link_libraries(controller PUBLIC ${PIGPIOD_LIBRARIES})
endif()
ament_target_dependencies(controller PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  backward_ros
)

# Чтобы ROS2-control мог загрузить компонент как плагин
pluginlib_export_plugin_description_file(hardware_interface controller_hw_interface.xml)

# ==============================
# Install
# ==============================

# Install executables
install(
  TARGETS controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install includes
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config directory
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install plugin description XML
install(
  FILES controller_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()