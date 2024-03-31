cmake_minimum_required(VERSION 3.8)
project(ros2_relbot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Add executable
add_executable(relbot_control src/RELbotControl.cpp)

ament_target_dependencies(
  relbot_control
  rclcpp
  sensor_msgs
  geometry_msgs
  custom_msg
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

# Install executable
install(TARGETS
  relbot_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
