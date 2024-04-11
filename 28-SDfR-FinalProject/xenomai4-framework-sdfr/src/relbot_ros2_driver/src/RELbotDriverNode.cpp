#include "relbot_ros2_driver/RELbotDriverNode.hpp"

RELbotDriverNode::RELbotDriverNode() : rclcpp::Node("relbot_driver")
{
    // Define ROS2 parameters and populate them with defaults if they are not overridden by run/launch configuration
    this->declare_parameter<double>("max_velocity", RELbotDriverNode::DEFAULT_MAX_VELOCITY);
    this->declare_parameter<double>("wheel_base_width", RELbotDriverNode::DEFAULT_WHEEL_BASE_WIDTH);
    this->declare_parameter<double>("wheel_radius", RELbotDriverNode::DEFAULT_WHEEL_RADIUS);

    // Create subscriptions and publishers
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic %s", this->CMD_VEL_TOPIC.c_str());
    cmdVelSubscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        this->CMD_VEL_TOPIC, 10, std::bind(&RELbotDriverNode::cmdVelCallback, this, std::placeholders::_1));

    const std::string right_motor_setpoint_vel_topic = this->RIGHT_MOTOR_NAMESPACE + this->SETPOINT_VEL_TOPIC;
    RCLCPP_INFO(this->get_logger(), "Subscribing publisher on topic %s", right_motor_setpoint_vel_topic.c_str());
    rightMotorSetpointVelSubscriber_ = this->create_subscription<example_interfaces::msg::Float64>(
        right_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotDriverNode::rightMotorSetpointVelCallback, this, std::placeholders::_1));

    const std::string left_motor_setpoint_vel_topic = this->LEFT_MOTOR_NAMESPACE + this->SETPOINT_VEL_TOPIC;
    RCLCPP_INFO(this->get_logger(), "Subscribing publisher on topic %s", left_motor_setpoint_vel_topic.c_str());
    leftMotorSetpointVelSubscriber_ = this->create_subscription<example_interfaces::msg::Float64>(
        left_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotDriverNode::leftMotorSetpointVelCallback, this, std::placeholders::_1));

    // Required publishers
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic %s", this->ROS2XENO_TOPIC.c_str());
    ros2xenoPublisher_ = this->create_publisher<custom_msgs::msg::Ros2Xeno>(this->ROS2XENO_TOPIC, 10);

    // Done
    RCLCPP_INFO(this->get_logger(), "Fully initialized, waiting on velocity messages");
}

void RELbotDriverNode::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmdVel)
{
    // Extract the linear x velocity and angular z velocity from the twist
    // Ignore any velocities on other axes
    float linearVel = cmdVel->twist.linear.x;
    float angularVel = cmdVel->twist.angular.z;
    RCLCPP_INFO(this->get_logger(), "Received twist setpoint linear.x = %f, angular.z = %f", linearVel, angularVel);

    // Compute motor velocity setpoints according to
    // http://wiki.ros.org/diff_drive_controller#Mathematical_Background
    // Also clamps the values so that the allowed speed of the motors is not exceeded
    rightMotorSetpointVel_ =
        std::clamp((linearVel + angularVel * this->get_parameter("wheel_base_width").as_double() / 2) /
                       this->get_parameter("wheel_radius").as_double(),
                   -this->get_parameter("max_velocity").as_double(), this->get_parameter("max_velocity").as_double());
    leftMotorSetpointVel_ =
        std::clamp((linearVel - angularVel * this->get_parameter("wheel_base_width").as_double() / 2) /
                       this->get_parameter("wheel_radius").as_double(),
                   -this->get_parameter("max_velocity").as_double(), this->get_parameter("max_velocity").as_double());

    this->publishMotorSetpointVelToXeno(leftMotorSetpointVel_, rightMotorSetpointVel_);
}

void RELbotDriverNode::rightMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel)
{
    // Clamp the setpoint velocity to the max velocity and store new setpoint
    // velocity internally
    RCLCPP_INFO(this->get_logger(), "Received right motor setpoint %f", setpointVel->data);
    rightMotorSetpointVel_ = std::clamp(setpointVel->data, -this->get_parameter("max_velocity").as_double(),
                                        this->get_parameter("max_velocity").as_double());

    // Set motor velocity, uses the last known value for the left motor setpoint
    this->publishMotorSetpointVelToXeno(leftMotorSetpointVel_, rightMotorSetpointVel_);
}

void RELbotDriverNode::leftMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel)
{
    // Clamp the setpoint velocity to the max velocity and store new setpoint
    // velocity internally

    RCLCPP_INFO(this->get_logger(), "Received left motor setpoint %f", setpointVel->data);
    leftMotorSetpointVel_ = std::clamp(setpointVel->data, -this->get_parameter("max_velocity").as_double(),
                                       this->get_parameter("max_velocity").as_double());

    // auto debugMsg = example_interfaces::msg::String();
    // debugMsg.data = "Test";
    // debugPublisher_->publish(debugMsg);

    // Set motor velocity, uses the last known value for the right motor setpoint
    this->publishMotorSetpointVelToXeno(leftMotorSetpointVel_, rightMotorSetpointVel_);
}

void RELbotDriverNode::publishMotorSetpointVelToXeno(double leftMotorSetpointVel, double rightMotorSetpointVel)
{
    auto ros2xenoMsg = custom_msgs::msg::Ros2Xeno();
    ros2xenoMsg.setpoint_vel_left = leftMotorSetpointVel;
    ros2xenoMsg.setpoint_vel_right = rightMotorSetpointVel;
    ros2xenoPublisher_->publish(ros2xenoMsg);
}

// Run the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RELbotDriverNode>());
    rclcpp::shutdown();
    return 0;
}
