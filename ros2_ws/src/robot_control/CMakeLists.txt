cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 build tools
find_package(ament_cmake REQUIRED)

# Install the folders so ROS 2 can find your files
install(DIRECTORY
  launch
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)
# Install the Python Node as an executable
install(PROGRAMS
  robot_control/robot_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

# Optional: If you created a config folder, uncomment the next line
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Optional: Install any python scripts (like kinematics_node.py) if they are in a scripts folder
# install(PROGRAMS
#   scripts/kinematics_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
