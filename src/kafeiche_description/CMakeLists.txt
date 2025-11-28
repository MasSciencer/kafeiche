cmake_minimum_required(VERSION 3.16)
project(kafeiche_description)

# Find the required packages
find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
