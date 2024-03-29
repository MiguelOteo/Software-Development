cmake_minimum_required(VERSION 3.8)
project(ros2_ball_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Add executable
add_executable(ball_detection src/BallDetection.cpp)

# Link dependencies
ament_target_dependencies(
  ball_detection
  rclcpp
  sensor_msgs
  custom_msg
  cv_bridge
  image_transport
  OpenCV
)

# Install executable
install(TARGETS
  ball_detection
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

ament_package()

