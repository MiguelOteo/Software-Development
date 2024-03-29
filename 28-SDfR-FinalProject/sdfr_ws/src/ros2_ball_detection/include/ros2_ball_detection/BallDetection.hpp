#ifndef BALL_DETECTION_HPP
#define BALL_DETECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "custom_msg/msg/bounding_box.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>

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
         * @brief Callback function for processing images received from ROS messages.
         * 
         * This function converts a ROS image message to an OpenCV Mat, performs ball detection
         * on the image, and publishes the detected image if debug visualization is enabled.
         *
         * @param msg The ROS image message received.
         */
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        /// Other methods
        /**
         * @brief Detects balls in the input image and draws bounding boxes around them.
         * 
         * This function takes an input image, converts it to the HSV color space, and segments
         * the image to detect balls based on predefined color ranges. It then performs morphological
         * operations to clean up the mask, finds contours of the detected balls, and draws bounding
         * boxes around them. Additionally, it publishes a message containing information about the
         * detected balls if any are found.
         *
         * @param image The input image in BGR color space.
         * @return A modified image with bounding boxes drawn around detected balls.
         */
        cv::Mat detect_balls(const cv::Mat& image);

        /// Private variables.
        bool debug_visualization_;

        /// Subscriber variables.
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

        /// Publisher variables.
        rclcpp::Publisher<custom_msg::msg::BoundingBox>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
};

#endif /* BALL_DETECTION_HPP */
