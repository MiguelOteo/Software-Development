cmake_minimum_required(VERSION 3.8)
project(ball_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(relbot_interfaces REQUIRED)
find_package(image_functions_sdfr REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

# Add executable
add_executable(ball_detection 
  src/BallDetection.cpp
  src/RGBtoHSV.cpp  # Add this line
)

# Link dependencies
ament_target_dependencies(
  ball_detection
  rclcpp
  sensor_msgs
  relbot_interfaces
  image_functions_sdfr
  image_transport
)

# Install executable
install(TARGETS
  ball_detection
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
