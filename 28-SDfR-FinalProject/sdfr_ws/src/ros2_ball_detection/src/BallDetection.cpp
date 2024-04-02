#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "custom_msg/msg/bounding_box.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <opencv2/opencv.hpp>
#include "../include/ros2_ball_detection/BallDetection.hpp"

/**
 * @brief Constructor for the BallDetection class.
 * 
 * This constructor initializes the BallDetection node, subscribing to the webcam image topic
 * for processing, creating publishers for bounding box information and debug images, and
 * initializing parameters such as debug visualization.
 */
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
    this->declare_parameter<bool>("debug_visualization", false);
}

/**
 * @brief Callback function for processing images received from ROS messages.
 * 
 * This function converts a ROS image message to an OpenCV Mat, performs ball detection
 * on the image, and publishes the detected image if debug visualization is enabled.
 *
 * @param msg The ROS image message received.
 */
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

    // Publish debug visualization if enabled  
    debug_visualization_ = this->get_parameter("debug_visualization").as_bool();

    if (debug_visualization_)
    {
        cv_ptr->image = detected_image;
        debug_publisher_->publish(*cv_ptr->toImageMsg());
    }
}

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

    if(contours.empty()) // Check if any contours were found if not skip cycle
    {
        publisher_->publish(std::move(bounding_box_msg)); 
        return image;
    } 

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
            bounding_box_msg->ball_found = true;
            bounding_box_msg->center_point_x = (bounding_rect.x + bounding_rect.width/2);
            bounding_box_msg->center_point_y = (bounding_rect.y + bounding_rect.height/2);
            bounding_box_msg->width = bounding_rect.width;
            bounding_box_msg->height = bounding_rect.height;

            // Draw the bounding box around the detected ball
            cv::rectangle(image, bounding_rect, cv::Scalar(101, 179, 129), 1);
        }
    }

    // Publish the message
    publisher_->publish(std::move(bounding_box_msg)); 

    return image;
}

/**
 * @brief Main function for initializing and running the BallDetection node.
 * 
 * This function initializes the ROS 2 node, creates an instance of the BallDetection class,
 * and starts spinning the node to handle ROS messages. Once the node finishes processing,
 * it shuts down the ROS 2 node and returns 0.
 *
 * @param argc The number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return An integer indicating the exit status.
 */
int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create and run the BallDetection node
    rclcpp::spin(std::make_shared<BallDetection>());
    
    // Shutdown ROS 2
    rclcpp::shutdown();
    
    // Return exit status
    return 0;
}
