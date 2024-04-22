#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include "example_interfaces/msg/float64.hpp"
#include "../include/relbot_motor_test/RELbotTestController.hpp"

using namespace std::chrono_literals;

RELbotTestController::RELbotTestController() : Node("relbot_test_controller")
{
    // Create a publisher for the right motor angular velocity 
    publisher_right_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/right_motor/setpoint_vel", 10);

    // Create a publisher for the left motor angular velocity 
    publisher_left_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/left_motor/setpoint_vel", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&RELbotTestController::timer_callback, this));
}

void RELbotTestController::timer_callback()
{
    auto omega_left = std::make_shared<example_interfaces::msg::Float64>();
    auto omega_right = std::make_shared<example_interfaces::msg::Float64>();

    omega_left->data = 0.5;
    omega_right->data = 0.2;

    publisher_right_vel_->publish(*omega_right);
    publisher_left_vel_->publish(*omega_left);
}

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create and run the RELbotControl node
    rclcpp::spin(std::make_shared<RELbotTestController>());
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    
    // Return exit status
    return 0;
}
