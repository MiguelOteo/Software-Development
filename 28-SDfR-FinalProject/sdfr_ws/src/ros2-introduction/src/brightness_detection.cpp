#include "../include/ros2_introduction/brightness_detection.hpp"
#include "custom_msg/msg/brightness_status.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_functions_sdfr/image_functions.hpp"
#include <iostream>
#include <string>

// Define the constructor for the BrightnessDetection class inheriting from 
// Node class with the node name "brightness_detection"
BrightnessDetection::BrightnessDetection() : Node("brightness_detection") 
{
    // Create a subscription to receive image data from the webcam topic "/image"
    subcription_ = this->create_subscription<sensor_msgs::msg::Image>
        ("/image", 10, std::bind(&BrightnessDetection::image_callback, this, std::placeholders::_1));

    // Create a publisher to publish the light level information on the topic "/light_level"
    light_level_publisher_ = this->create_publisher<example_interfaces::msg::Int8>
        ("/light_level", 10);

    // Create a publisher to publish the brightness status 
    // information on the topic "/brightness_status_custom"
    brightness_status_publisher_ = this->create_publisher<custom_msg::msg::BrightnessStatus> 
        ("/brightness_status_custom", 10);
}

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
    auto light_msg = std::make_unique<example_interfaces::msg::Int8>(); // Create a unique pointer for the Int8 message
    light_msg->data = avg_brightness; // Assign the calculated average brightness to the Int8 message data field
    light_level_publisher_->publish(std::move(light_msg)); // Publish the Int8 message

    // Determine the brightness status based on the calculated average brightness and a predefined threshold
    std::string brightness_status = (avg_brightness > threshold_) ? "Bright" : "Dark";

    // Publish the determined brightness status to the "/brightness_status" topic
    auto status_msg = std::make_unique<custom_msg::msg::BrightnessStatus>(); // Create a unique pointer for the BrightnessStatus message
    status_msg->brightness_status = brightness_status; 
    status_msg->light_level = avg_brightness;
    brightness_status_publisher_->publish(std::move(status_msg)); // Publish the String message
}


// The main function starts the node and "spins" it, i.e. handles all ROS2-related events 
// such as receiving messages on topics
// You rarely need to add anything else to this function for ROS2 nodes
int main(int argc, char * argv[]) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BrightnessDetection>());
    rclcpp::shutdown();
    return 0;
}
