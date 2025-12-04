cmake_minimum_required(VERSION 3.16)
project(kafeiche_webrtc_cam LANGUAGES CXX)

# ==============================
# C++ Standard & Settings
# ==============================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
# Find dependencies
# ==============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)

# GStreamer
pkg_check_modules(GSTREAMER REQUIRED 
  gstreamer-1.0 
  gstreamer-webrtc-1.0 
  gstreamer-sdp-1.0
  # если нужны bad plugins, раскомментируй:
  # gstreamer-plugins-bad-1.0
)

# ==============================
# Executable
# ==============================
add_executable(cam_webrtc_node 
  src/cam_webrtc_node.cpp
)

target_include_directories(cam_webrtc_node PRIVATE
  ${GSTREAMER_INCLUDE_DIRS}
  include
)

target_compile_options(cam_webrtc_node PRIVATE
  ${GSTREAMER_CFLAGS_OTHER}
)

# ВАЖНО: всё линкование только в keyword-стиле и в ОДНОМ месте
target_link_libraries(cam_webrtc_node PRIVATE
  rclcpp::rclcpp
  ${GSTREAMER_LIBRARIES}
  # если понадобятся bad-плагины:
  # ${GSTREAMER_PLUGINS_BAD_LIBRARIES}
)

# ==============================
# Install
# ==============================
install(TARGETS cam_webrtc_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()