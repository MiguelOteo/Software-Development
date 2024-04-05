#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "../include/relbot_diff_drive_control/DiffDriveController.hpp"

/**
 * @brief Constructor for the DiffDriveController class.
 * 
 * Initializes the DiffDriveController object. Sets up subscriptions to twist messages,
 * creates publishers for motor velocity commands, and initializes parameters.
 */
DiffDriveController::DiffDriveController() : Node("diff_driver_controller")
{
    /// RELbot drive control
    // Create subscriber for TwistStamped messages
    subscriber_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>
        ("/cmd_vel", 10, std::bind(&DiffDriveController::compute_diff_velocities, this, std::placeholders::_1));

    // Create a publisher for the right motor angular velocity 
    publisher_right_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/right_motor/setpoint_vel", 10);

    // Create a publisher for the left motor angular velocity 
    publisher_left_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/left_motor/setpoint_vel", 10);

}

/**
 * @brief Callback function for computing differential velocities.
 * 
 * This function computes the angular velocities for the left and right wheels
 * based on linear and angular velocity inputs and publishes them to control
 * the robot's motion.
 *
 * @param rebot_twists A shared pointer to the received TwistStamped message.
 */
void DiffDriveController::compute_diff_velocities(const geometry_msgs::msg::TwistStamped::SharedPtr rebot_twists)
{
    // Init all the messages
    auto omega_left = std::make_shared<example_interfaces::msg::Float64>();
    auto omega_right = std::make_shared<example_interfaces::msg::Float64>();

    float linear_velocity = rebot_twists->twist.linear.x;
    float angular_velocity = rebot_twists->twist.angular.z;

    // Calculate target angular velocities for both wheels
    double wheel_base = 0.5; // distance between the wheels (in meters)
    double wheel_radius = 0.1; // radius of the wheels (in meters)
    omega_left->data = (linear_velocity - (angular_velocity * wheel_base / 2)) / wheel_radius;
    omega_right->data = (linear_velocity + (angular_velocity * wheel_base / 2)) / wheel_radius;

    publisher_right_vel_->publish(*omega_right);
    publisher_left_vel_->publish(*omega_left);
}

/**
 * @brief Main function for the DiffDriveController node.
 * 
 * Initializes ROS 2, creates and runs the DiffDriveController node, and then 
 * shuts down ROS 2. Returns the exit status of the program.
 * 
 * @param argc The number of command-line arguments.
 * @param argv An array of strings containing the command-line arguments.
 * @return The exit status of the program.
 */
int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create and run the DiffDriveController node
    rclcpp::spin(std::make_shared<DiffDriveController>());
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    
    // Return exit status
    return 0;
}
