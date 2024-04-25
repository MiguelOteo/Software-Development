#include <vector>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "relbot_interfaces/msg/bounding_box.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "image_functions_sdfr/image_functions.hpp"
#include "../include/ros2_ball_detection/BallDetection.hpp"
#include "../include/ros2_ball_detection/BGRtoHSV.hpp"
#include "../include/ros2_ball_detection/BoundingBoxDetection.hpp"

// Define the color for ball detection and drawn squares
const int RGB_BOX[3] = {108, 142, 191};

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
    camimage_subscription_ = this->create_subscription<sensor_msgs::msg::Image>
        ("/image",10,std::bind(&BallDetection::ball_detection_callback, this, std::placeholders::_1));

    // Publish the bounding box information
    bounding_box_publisher_ = this->create_publisher<relbot_interfaces::msg::BoundingBox>
        ("/bounding_box",10);

    // Publish debug images
    debug_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>
        ("/debug_image",10);

    // Initialize parameters
    this->declare_parameter<bool>("debug_visualization", false);
}

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
void BallDetection::ball_detection_callback(const sensor_msgs::msg::Image::SharedPtr image)
{
    // Get the size of the drawing area from the webcam image
    int sizeX = image_functions::getImageWidth(image); 
    int sizeY = image_functions::getImageHeight(image);

    // Get the number of threads of the system CPU
    const int num_threads = std::thread::hardware_concurrency();

    RCLCPP_INFO_ONCE(this->get_logger(), "Using %d threads", num_threads);

    // Define a vector to hold all the thread objects
    std::vector<std::thread> thread_pool(num_threads);

    // Pair to store the x and y coordinates of the ball pixels
    std::pair<std::vector<int>, std::vector<int>> ball_pixels;

    // Launch the threads to detect the pixels of the ball form 
    // the assigned region of the camera
    for(int thread_num = 0; thread_num < num_threads; thread_num++)
    {
        thread_pool[thread_num] = 
            std::thread(ball_pixels_detection, 
                        thread_num, 
                        num_threads, 
                        sizeX, 
                        sizeY, 
                        image, 
                        std::ref(ball_pixels));
    }

    // Join all the threads
    for(auto& thread : thread_pool)
    {
        thread.join();  
    }

    // Turn the pixels of the ball into a bounding box message
    relbot_interfaces::msg::BoundingBox::SharedPtr bounding_box_msg = 
        get_bounding_box(ball_pixels);

    // Check if debug visualization is enabled
    bool debug_visualization = this->get_parameter("debug_visualization").as_bool();

    // If debug visualization is enabled, create and publish a debug image with the bounding box drawn around the detected ball
    if(debug_visualization)
    {
        publish_debug_image(image, bounding_box_msg);
    }

    // Publish the detected bounding box
    bounding_box_publisher_->publish(std::move(*bounding_box_msg)); 
}

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
void BallDetection::publish_debug_image
    (const sensor_msgs::msg::Image::SharedPtr image, 
     relbot_interfaces::msg::BoundingBox::SharedPtr bounding_box_msg)
{
    // If ball not found skip the drawing 
    if(!bounding_box_msg->ball_found)
    {
        debug_image_publisher_->publish(std::move(*image)); 
        return;
    }

    // Create a new image for debug purposes with the same properties as the original image
    sensor_msgs::msg::Image::SharedPtr debug_image = std::make_shared<sensor_msgs::msg::Image>();  
    image_functions::copyImageProperties(debug_image, image);

    // Get the coordinates of the bounding box
    int min_x = bounding_box_msg->center_point_x - bounding_box_msg->width/2;
    int max_x = bounding_box_msg->center_point_x + bounding_box_msg->width/2;
    int min_y = bounding_box_msg->center_point_y - bounding_box_msg->height/2;
    int max_y = bounding_box_msg->center_point_y + bounding_box_msg->height/2;

    // Draw a bounding box on the debug image
    for (int x = min_x; x < max_x; x++)
    {
        // If not first or last column then draw only top and bottom lines
        if(x != min_x && x != max_x-1)
        {
            image_functions::setPixelColor(debug_image, x, min_y, RGB_BOX[2], RGB_BOX[1], RGB_BOX[0]);
            image_functions::setPixelColor(debug_image, x, max_y-1, RGB_BOX[2], RGB_BOX[1], RGB_BOX[0]);
            continue;
        }        

        // Draw vertical lines of the bounding box
        for (int y = min_y; y < max_y; y++)
        {
            image_functions::setPixelColor(debug_image, x, y, RGB_BOX[2], RGB_BOX[1], RGB_BOX[0]);
        }
    }

    // Publish the debug image with the drawn bounding box
    debug_image_publisher_->publish(std::move(*debug_image)); 
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
