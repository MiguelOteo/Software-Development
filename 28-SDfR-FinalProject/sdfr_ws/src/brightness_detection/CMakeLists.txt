cmake_minimum_required(VERSION 3.8)
project(brightness_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(image_functions_sdfr REQUIRED)
find_package(relbot_interfaces REQUIRED)
include_directories(include)

# Add brightness_detection executable
add_executable(brightness_detection src/BrightnessDetection.cpp)

ament_target_dependencies(
  brightness_detection 
  rclcpp 
  sensor_msgs 
  example_interfaces 
  image_functions_sdfr 
  relbot_interfaces
)

# Add custom_brightness_detection executable
add_executable(custom_brightness_detection src/CustomBrightnessDetection.cpp)

ament_target_dependencies(
  custom_brightness_detection
  rclcpp 
  sensor_msgs 
  example_interfaces 
  image_functions_sdfr 
  relbot_interfaces
)

# Install targets
install(
  TARGETS 
    brightness_detection
    custom_brightness_detection
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
