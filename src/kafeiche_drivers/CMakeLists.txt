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
# WiringPi library (for servo motors)
# ==============================
find_library(WIRINGPI_LIBRARIES wiringPi)
if(NOT WIRINGPI_LIBRARIES)
  message(FATAL_ERROR "WiringPi library not found!")
endif()

# ==============================
# Include dirs
# ==============================
include_directories(include)

# ==============================
# servo_motor executable
# ==============================
add_executable(servo_motor src/servo_motor.cpp)
target_link_libraries(servo_motor ${WIRINGPI_LIBRARIES})
ament_target_dependencies(
  servo_motor
  rclcpp
  std_msgs
)

# Add shared library for hardware interface plugin
add_library(
  controller
  SHARED
  src/controller.cpp
)
ament_target_dependencies(controller PUBLIC
  hardware_interface
  pluginlib
  std_msgs
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
  TARGETS servo_motor controller
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