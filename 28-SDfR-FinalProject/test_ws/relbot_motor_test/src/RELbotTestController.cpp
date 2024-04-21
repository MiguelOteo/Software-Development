#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "../include/relbot_motor_test/RELbotTestController.hpp"

RELbotTestController::RELbotTestController() : Node("relbot_test_controller")
{
    // Create a publisher for the right motor angular velocity 
    publisher_right_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/right_motor/setpoint_vel", 10);

    // Create a publisher for the left motor angular velocity 
    publisher_left_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/left_motor/setpoint_vel", 10);
}