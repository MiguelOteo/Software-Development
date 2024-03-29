#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "custom_msg/msg/bounding_box.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>
#include "../include/ros2_ball_detection/BallDetection.hpp"

BallDetection::BallDetection() : Node("ball_detection")
{
    // Subscribe to the webcam image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>
        ("/image",10,std::bind(&BallDetection::image_callback, this, std::placeholders::_1));

    // Publish the bounding box information
    publisher_ = this->create_publisher<custom_msg::msg::BoundingBox>
        ("/bounding_box",10);

    // Publish debug images
    debug_publisher_ = this->create_publisher<sensor_msgs::msg::Image>
        ("/debug_image",10);

    // Initialize parameters
    this->declare_parameter<bool>("debug_visualization", true);
    this->get_parameter("debug_visualization", debug_visualization_);
}

void BallDetection::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
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

    // Perform ball detection
    cv::Mat detected_image = detect_balls(cv_ptr->image);
    cv_ptr->image = detected_image;

    // Publish debug visualization if enabled
    if (debug_visualization_)
    {
        debug_publisher_->publish(*cv_ptr->toImageMsg());
    }
}

cv::Mat BallDetection::detect_balls(const cv::Mat& image)
{
    // Convert image to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Define the range of colors for the ball (you may need to adjust these values)
    cv::Scalar lower_bound(30, 50, 50); // Lower bound for HSV (greenish color)
    cv::Scalar upper_bound(90, 255, 255); // Upper bound for HSV (greenish color)

    // Create a mask to segment the ball
    cv::Mat mask;
    cv::inRange(hsv_image, lower_bound, upper_bound, mask);

    // Perform morphological operations to clean up the mask
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat());
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat());

    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Prepare message to publish
    auto bounding_box_msg = std::make_unique<custom_msg::msg::BoundingBox>();
    bounding_box_msg->ball_found = !contours.empty(); // Check if any contours were found

    // Draw bounding boxes around detected balls
    for (const auto& contour : contours)
    {
        // Fit a bounding rectangle around the contour
        cv::Rect bounding_rect = cv::boundingRect(contour);
        
        // Calculate the aspect ratio to filter out non-circular shapes
        double aspect_ratio = static_cast<double>(bounding_rect.width) / bounding_rect.height;
        
        // Check if the aspect ratio is close to 1 (circular shape)
        if (aspect_ratio >= 0.9 && aspect_ratio <= 1.1)
        {
            // Fill in message data
            bounding_box_msg->center_point_x = (bounding_rect.x + bounding_rect.width) / 2.0;
            bounding_box_msg->center_point_y = (bounding_rect.y + bounding_rect.height) / 2.0;
            bounding_box_msg->width = bounding_rect.width;
            bounding_box_msg->height = bounding_rect.height;

            // Draw the bounding box around the detected ball
            cv::rectangle(image, bounding_rect, cv::Scalar(0, 255, 0), 2);
        }
    }

    // Publish the message
    publisher_->publish(std::move(bounding_box_msg)); 

    return image;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallDetection>());
    rclcpp::shutdown();
    return 0;
}
