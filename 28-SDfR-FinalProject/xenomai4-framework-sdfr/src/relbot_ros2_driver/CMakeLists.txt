cmake_minimum_required(VERSION 3.10)
project(relbot_ros2_driver)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)


# RELbot driver node
add_executable(relbot_driver
  src/RELbotDriverNode.cpp
)
target_include_directories(relbot_driver
  PUBLIC include
)
ament_target_dependencies(relbot_driver
  PUBLIC
  rclcpp
  geometry_msgs
  example_interfaces
  custom_msgs
)

# Install all executables (nodes)
install(TARGETS
  relbot_driver
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Populate share directory (to include launch files)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
