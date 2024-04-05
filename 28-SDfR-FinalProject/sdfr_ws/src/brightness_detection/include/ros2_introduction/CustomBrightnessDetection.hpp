#ifndef CUSTOM_BRIGHTNESS_DETECTION_HPP
#define CUSTOM_BRIGHTNESS_DETECTION_HPP

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "example_interfaces/msg/string.hpp"
#include "relbot_interfaces/msg/brightness_status.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

/**
 * @brief Class for detecting brightness level in images and publishing status.
 * 
 * The BrightnessDetection class is responsible for processing images received from ROS messages,
 * detecting brightness levels in the images, and publishing status information. It subscribes to
 * a ROS topic for receiving images, processes them to determine brightness levels, and publishes
 * status information based on the detected brightness levels.
 */
class CustomBrightnessDetection : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the CustomBrightnessDetection class.
     * 
     * This constructor initializes the CustomBrightnessDetection node, creating subscriptions for image
     * messages, and publishers for publishing brightness level and status messages.
     */
    CustomBrightnessDetection();

private:
    /// Callback functions.
    /**
     * @brief Callback function for processing images received from ROS messages.
     * 
     * This function calculates the average brightness of the received image using helper functions,
     * publishes the calculated average brightness to the "/light_level" topic, determines the brightness
     * status based on a predefined threshold, and publishes the determined brightness status to the
     * "/brightness_status" topic.
     *
     * @param img The image that was received.
     */
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr img);

    /// Private variables.
    int threshold_ = 100; // Example threshold value, you should define the appropriate threshold for your application.

    /// Subscriber variables.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camimage_subscription_;

    /// Publisher variables.
    rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr light_level_publisher_;
    rclcpp::Publisher<relbot_interfaces::msg::BrightnessStatus>::SharedPtr brightness_status_publisher_;

};

#endif /* CUSTOM_BRIGHTNESS_DETECTION_HPP */
