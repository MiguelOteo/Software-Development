#ifndef RELBOT_ICO_ROS2_RELBOTDRIVERNODE
#define RELBOT_ICO_ROS2_RELBOTDRIVERNODE

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "custom_msgs/msg/ros2_xeno.hpp"

/**
 * @brief RELbot driver node. This node converts setpoint velocities into individual motor command velocities and 
 * forwards them to the RosXeno bridge.
 *
 * @param max_velocity [double]: Max allowed velocity for individual motors in rad/s (in positive and negative 
 * direction). If any velocity higher than this is reached, it is clamped to the max velocity instead. Note that the 
 * velocity might be further limited by other components. Default value: 1.0
 * @param wheel_base_width [double]: Width in meters between the robot's powered wheels. Default value: 0.209
 * @param wheel_radius [double]: Wheel radius in meters for the robot's powered wheels. Default value: 0.05
 */
class RELbotDriverNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Differential Drive Controller node.
     */
    RELbotDriverNode();

    // Default parameter values
    const double DEFAULT_MAX_VELOCITY = 1.0;        // [rad/s] Max allowed velocity of individual motors
    const double DEFAULT_WHEEL_BASE_WIDTH = 0.209;  // [m] Width between the center of both wheels
    const double DEFAULT_WHEEL_RADIUS = 0.05;       // [m] Radius of the wheels

    // Default topic names
    const std::string CMD_VEL_TOPIC = "/cmd_vel";
    const std::string RIGHT_MOTOR_NAMESPACE = "/right_motor";
    const std::string LEFT_MOTOR_NAMESPACE = "/left_motor";
    const std::string SETPOINT_VEL_TOPIC = "/setpoint_vel";
    const std::string ROS2XENO_TOPIC = "/Ros2Xeno";

private:
    // Required subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelSubscriber_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr rightMotorSetpointVelSubscriber_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr leftMotorSetpointVelSubscriber_;

    // Required publishers
    rclcpp::Publisher<custom_msgs::msg::Ros2Xeno>::SharedPtr ros2xenoPublisher_;

    // Stored motor velocity setpoints
    double rightMotorSetpointVel_ = 0.0;
    double leftMotorSetpointVel_ = 0.0;

    /**
     * @brief Callback upon receiving a command velocity. Converts twist into two motor commands and sets them
     * immediately
     */
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmdVel);

    /**
     * @brief Callback upon receiving the right motor command velocity. Sets the new motor velocity with the new right
     * motor value and the last received left motor value.
     */
    void rightMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel);

    /**
     * @brief Callback upon receiving the left motor command velocity. Sets the new motor velocity with the new left
     * motor value and the last received right motor value.
     */
    void leftMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel);

    /**
     * @brief Publish motor setpoint velocity [rad/s] to the Ros2Xenomai bridge.
     */
    void publishMotorSetpointVelToXeno(double leftMotorSetpointVel, double rightMotorSetpointVel);
};

#endif // RELBOT_ICO_ROS2_RELBOTDRIVERNODE
