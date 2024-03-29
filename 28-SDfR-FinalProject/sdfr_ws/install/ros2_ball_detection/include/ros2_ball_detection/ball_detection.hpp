#ifndef BALL_DETECTION_HPP
#define BALL_DETECTION_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "custom_msg/msg/bounding_box.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>

class ball_detection: public rclcpp::Node
{
    public:
        ball_detection();

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        cv::Mat detect_balls(const cv::Mat& image);
        //void publish_debug_image(const cv::Mat& image);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Publisher<custom_msg::msg::BoundingBox>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
        bool debug_visualization_;
};

#endif /* BALL_DETECTION_HPP */
