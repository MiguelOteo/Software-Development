#include "../include/ros2_introduction/brightness_detection.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "example_interfaces/msg/string.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "image_functions_sdfr/image_functions.hpp"
#include <iostream>
#include <string>

/**
 * @brief Constructor for the BrightnessDetection class.
 * 
 * This constructor initializes the BrightnessDetection node, creating subscriptions for image
 * messages, and publishers for publishing brightness level and status messages.
 */
BrightnessDetection::BrightnessDetection() : Node("brightness_detection") 
{
    // Create a subscription to receive image data from the webcam topic "/image"
    camimage_subscription_ = this->create_subscription<sensor_msgs::msg::Image>
        ("/image", 10, std::bind(&BrightnessDetection::image_callback, this, std::placeholders::_1));

    // Create a publisher to publish the light level information on the topic "/light_level"
    light_level_publisher_ = this->create_publisher<example_interfaces::msg::UInt8>
        ("/light_level", 10);

    // Create a publisher to publish the brightness status 
    // information on the topic "/brightness_status_custom"
    brightness_status_publisher_ = this->create_publisher<example_interfaces::msg::String> 
        ("/brightness_status", 10);
}

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
void BrightnessDetection::image_callback(sensor_msgs::msg::Image::ConstSharedPtr img) 
{
    // Calculate the average brightness of the received image using a helper function
    int avg_brightness = 0;
    int total_brightness = 0;
    int num_pixels = image_functions::getImageWidth(img) * image_functions::getImageHeight(img);

    for (int y = 0; y < image_functions::getImageHeight(img); ++y) {
        for (int x = 0; x < image_functions::getImageWidth(img); ++x) {
            total_brightness += image_functions::getPixelBrightness(img, x, y);
        }
    }
    
    avg_brightness = total_brightness / num_pixels;

    // Publish the calculated average brightness to the "/light_level" topic
    auto light_msg = std::make_unique<example_interfaces::msg::UInt8>(); // Create a unique pointer for the Int8 message
    light_msg->data = avg_brightness; // Assign the calculated average brightness to the Int8 message data field
    light_level_publisher_->publish(std::move(light_msg)); // Publish the Int8 message

    // Determine the brightness status based on the calculated average brightness and a predefined threshold
    std::string brightness_status = (avg_brightness > threshold_) ? "Bright" : "Dark";

    // Publish the determined brightness status to the "/brightness_status" topic
    auto status_msg = std::make_unique<example_interfaces::msg::String>();
    status_msg->data = brightness_status; 
    brightness_status_publisher_->publish(std::move(status_msg)); // Publish the BrightnessStatus message
}

/**
 * @brief Main function for initializing and running the BrightnessDetection node.
 * 
 * This function initializes the ROS 2 node, creates an instance of the BrightnessDetection class,
 * and starts spinning the node to handle ROS messages. Once the node finishes processing,
 * it shuts down the ROS 2 node and returns 0.
 *
 * @param argc The number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return An integer indicating the exit status.
 */
int main(int argc, char * argv[]) 
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create and run the BrightnessDetection node
    rclcpp::spin(std::make_shared<BrightnessDetection>());
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    
    // Return exit status
    return 0;
}

