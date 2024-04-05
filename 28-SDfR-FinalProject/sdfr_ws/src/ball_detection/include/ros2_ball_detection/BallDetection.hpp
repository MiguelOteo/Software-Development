#ifndef BALL_DETECTION_HPP
#define BALL_DETECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "relbot_interfaces/msg/bounding_box.hpp"
#include "sensor_msgs/image_encodings.hpp"

/**
 * @brief Class for detecting balls in images and publishing bounding box information.
 * 
 * The BallDetection class is responsible for processing images received from ROS messages,
 * detecting balls in the images, and publishing bounding box information about the detected
 * balls. It subscribes to a ROS topic for receiving images, performs ball detection using
 * computer vision techniques, and publishes information about the detected balls.
 */
class BallDetection: public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the BallDetection class.
         * 
         * This constructor initializes the BallDetection node, subscribing to the webcam image topic
         * for processing, creating publishers for bounding box information and debug images, and
         * initializing parameters such as debug visualization.
         */
        BallDetection();

    private:
        /// Callback functions.
        /**
         * @brief Callback function triggered upon receiving an image from the webcam for ball detection.
         * 
         * This function performs ball detection algorithm on the received image, 
         * publishes a debug image if debug visualization is enabled, and publishes the 
         * detected bounding box of the ball.
         * 
         * @param image A shared pointer to sensor_msgs::msg::Image 
         *  message containing the image data from the webcam.
         */
        void ball_detection_callback
            (const sensor_msgs::msg::Image::SharedPtr image);

        /**
         * @brief Publishes a debug image with a bounding box drawn around the detected ball.
         * 
         * If the ball is not found in the bounding box message, the original 
         * image is published without any modifications.
         * 
         * @param image A shared pointer to sensor_msgs::msg::Image 
         *  message containing the original image data.
         * @param bounding_box_msg A shared pointer to relbot_interfaces::msg::BoundingBox 
         *  message representing the detected bounding box of the ball.
         */
        void publish_debug_image
            (const sensor_msgs::msg::Image::SharedPtr image, 
             relbot_interfaces::msg::BoundingBox::SharedPtr bounding_box_msg);

        /// Private variables.
        bool debug_visualization_;

        /// Subscriber variables.
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camimage_subscription_;

        /// Publisher variables.
        rclcpp::Publisher<relbot_interfaces::msg::BoundingBox>::SharedPtr bounding_box_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_publisher_;
};

#endif /* BALL_DETECTION_HPP */
