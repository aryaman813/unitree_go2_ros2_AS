cmake_minimum_required(VERSION 3.5)
project(b2_description)

# Find ROS 2 build tools
find_package(ament_cmake REQUIRED)

# Install directories (urdf, meshes, launch, etc.)
install(
  DIRECTORY launch meshes urdf xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
