#ifndef BRIGHTNESS_DETECTION_HPP
#define BRIGHTNESS_DETECTION_HPP

#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
// Define message includes here...

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
    // ...

    /// Subscriber variables.
    // ...

    /// Publisher variables.
    // ...
};

#endif /* BRIGHTNESS_DETECTION_HPP */