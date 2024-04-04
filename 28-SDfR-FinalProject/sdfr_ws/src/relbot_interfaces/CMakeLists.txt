cmake_minimum_required(VERSION 3.5)
project(relbot_interfaces)

# Enable C++11 or newer
set(CMAKE_CXX_STANDARD 11)

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(rosidl_default_generators REQUIRED)

# Generate ROS 2 interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BrightnessStatus.msg"
  "msg/BoundingBox.msg"
)

# Optionally, include testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif() 

# Package installation
ament_package()
