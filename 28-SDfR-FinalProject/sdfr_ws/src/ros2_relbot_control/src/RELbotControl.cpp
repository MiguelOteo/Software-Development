#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/bounding_box.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "example_interfaces/msg/float64.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "../include/ros2_relbot_control/RELbotControl.hpp"

/**
 * @brief Constructor for the RELbotControl class.
 * 
 * Initializes the RELbotControl object. Sets up subscriptions to topics 
 * for bounding box and image messages, creates publishers for twist commands 
 * and debug images, and initializes parameters.
 */
RELbotControl::RELbotControl() : Node("relbot_control")
{
    /// RELbot control
    // Subscribe to the /bounding_box topic
    subscription_box_ = this->create_subscription<custom_msg::msg::BoundingBox>
        ("/bounding_box", 10, std::bind(&RELbotControl::control_callback, this, std::placeholders::_1));

    // Subscribe to the /image topic to update the center of the reference bounding box
    subscription_image_ = this->create_subscription<sensor_msgs::msg::Image>
        ("/image",10,std::bind(&RELbotControl::bounding_box_center, this, std::placeholders::_1));

    // Create publisher for TwistStamped messages
    publisher_twist_ = this->create_publisher<geometry_msgs::msg::TwistStamped>
        ("/cmd_vel", 10);

    // Create a publisher for the right motor angular velocity 
    publisher_right_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/right_motor/setpoint_vel", 10);

    // Create a publisher for the left motor angular velocity 
    publisher_left_vel_ = this->create_publisher<example_interfaces::msg::Float64>
        ("/input/left_motor/setpoint_vel", 10);

    /// Image debug
    // Subscriber to \debug_image topic 
    debug_subscription_ = this->create_subscription<sensor_msgs::msg::Image>
        ("/debug_image",10,std::bind(&RELbotControl::debug_image_callback, this, std::placeholders::_1));

    // Publish debug images
    debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>
        ("/debug_image_control",10);

    // Initialize parameters
    this->declare_parameter<bool>("debug_visualization", false);
    this->declare_parameter<double>("linear_gain", 1.0);
    this->declare_parameter<double>("angular_gain", 1.0);
    this->declare_parameter<int>("ball_size", 50.0);
}

/**
 * @brief Callback function for controlling the robot based on bounding box information.
 * 
 * This function is called whenever a new bounding box message is received. 
 * It computes the control commands based on the bounding box information and publishes 
 * TwistStamped messages to control the robot's motion.
 *
 * @param bounding_box A shared pointer to the bounding box message.
 */
void RELbotControl::control_callback(const custom_msg::msg::BoundingBox::SharedPtr bounding_box)
{
    // Init all the messages
    auto twist_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
    auto omega_left = std::make_shared<example_interfaces::msg::Float64>();
    auto omega_right = std::make_shared<example_interfaces::msg::Float64>();

    // Check for parameters updates
    reference_bounding_box.height = this->get_parameter("ball_size").as_int();
    reference_bounding_box.width = this->get_parameter("ball_size").as_int();
    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();

    // If ball found then compute speed
    if(bounding_box->ball_found)
    {
        // Calculate errors
        double size_error = reference_bounding_box.width - bounding_box->width;
        double center_error = reference_bounding_box.center_point_x - bounding_box->center_point_x; 

        // Calculate command velocities
        double linear_velocity = linear_gain_ * size_error;
        double angular_velocity = angular_gain_ * center_error;

        // Calculate target angular velocities for both wheels
        double wheel_base = 0.5; // distance between the wheels (in meters)
        double wheel_radius = 0.1; // radius of the wheels (in meters)
        omega_left->data = (linear_velocity - (angular_velocity * wheel_base / 2)) / wheel_radius;
        omega_right->data = (linear_velocity + (angular_velocity * wheel_base / 2)) / wheel_radius;

        // Add data to the TwistStamped message
        twist_msg->header.stamp = this->now();
        twist_msg->twist.linear.x = linear_velocity;
        twist_msg->twist.angular.z = angular_velocity;
    }

    // Publish TwistStamped message
    publisher_twist_->publish(*twist_msg);

    // Publish individual motor commands (angular velocities)
    publisher_right_vel_->publish(*omega_right);
    publisher_left_vel_->publish(*omega_left);
}

/**
 * @brief Callback function for updating the center of the reference bounding box.
 * 
 * This function is called whenever a new image message is received. It updates the 
 * center point of the reference bounding box based on the resolution of the received image.
 *
 * @param msg A shared pointer to the image message.
 */
void RELbotControl::bounding_box_center(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Updates the center of the webcam based on the resolution of the webcam input
    reference_bounding_box.center_point_x = (msg->width)/2;
    reference_bounding_box.center_point_y = (msg->height)/2;
}

/**
 * @brief Callback function for processing debug images.
 * 
 * This function is called whenever a new debug image message is received. 
 * If debug visualization is enabled, it converts the received ROS image message 
 * to an OpenCV Mat, draws a rectangle overlay on the image representing the 
 * reference bounding box, and publishes the debug image.
 *
 * @param msg A shared pointer to the debug image message.
 */
void RELbotControl::debug_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Publish debug visualization if enabled
    debug_visualization_ = this->get_parameter("debug_visualization").as_bool();

    if(!debug_visualization_)
    {
        return;
    }

    // Convert ROS image message to OpenCV Mat
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Draw the rectangle overlay
    cv::rectangle(cv_ptr->image,
        cv::Point(reference_bounding_box.center_point_x - reference_bounding_box.width / 2, 
                  reference_bounding_box.center_point_y - reference_bounding_box.height / 2),
        cv::Point(reference_bounding_box.center_point_x + reference_bounding_box.width / 2, 
                  reference_bounding_box.center_point_y + reference_bounding_box.height / 2),
        cv::Scalar(191, 142, 108), 1);

    // Publish the debug image with the detected contour and the reference one
    debug_publisher_->publish(*cv_ptr->toImageMsg());
}

/**
 * @brief Main function for the RELbotControl node.
 * 
 * Initializes ROS 2, creates and runs the RELbotControl node, and then 
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
    
    // Create and run the RELbotControl node
    rclcpp::spin(std::make_shared<RELbotControl>());
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    
    // Return exit status
    return 0;
}
