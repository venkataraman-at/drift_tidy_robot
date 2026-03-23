cmake_minimum_required(VERSION 3.8)
project(drift_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  worlds
  launch
  config
  meshes
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/navigator.py
  scripts/pick_place.py
  scripts/dummy_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navigator
)
ament_package()
