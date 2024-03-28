#ifndef BRIGHTNESS_DETECTION_HPP
#define BRIGHTNESS_DETECTION_HPP

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "example_interfaces/msg/int8.hpp"
#include "example_interfaces/msg/string.hpp"

// Placeholder for std::bind.
using std::placeholders::_1;

class BrightnessDetection : public rclcpp::Node {
public:
    BrightnessDetection();

private:
    /// Callback functions.
    /**
     * @brief Callback to process any incoming Image messages.
     * 
     * @param img The image that was received.
    */
    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr img);

    /// Private variables.
    // Define private variables here if needed.
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subcription_;
    rclcpp::Publisher<example_interfaces::msg::Int8>::SharedPtr light_level_publisher_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr brightness_status_publisher_;

    int threshold_ = 100; // Example threshold value, you should define the appropriate threshold for your application.

    /// Subscriber variables.
    // Define subscriber variables here if needed.

    /// Publisher variables.
    // Define publisher variables here if needed.
};

#endif /* BRIGHTNESS_DETECTION_HPP */
