#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "relbot_interfaces/msg/bounding_box.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "image_functions_sdfr/image_functions.hpp"
#include "../include/ros2_ball_detection/BallDetection.hpp"
#include "../include/ros2_ball_detection/RGBtoHSV.hpp"

// Define the color for ball detection and drawn squares
const int RGB_BOX[3] = {108, 142, 191};

// Define the upper and lower bound in the HSV space
// Hue [0-360] Red around 0 | Green at 120 | Blue at 240
// Stauration [0-1]
// Value [0-1]
const float LOWER_BOUND_HSV[3] = {70, 0.3, 0.3};
const float UPPER_BOUND_HSV[3] = {160, 1, 1};

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

    // Perform ball detection algorithm to find the bounding box
    relbot_interfaces::msg::BoundingBox::SharedPtr bounding_box_msg = 
        ball_detection_algorithm(sizeX, sizeY, image);

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
        // If first or last column then draw only top and bottom lines
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
 * @brief Main function for ball detection algorithm.
 * 
 * This function takes in the dimensions of the image and the image data,
 * and returns a shared pointer to a BoundingBox message indicating the detected ball's location.
 * 
 * @param sizeX The width of the image.
 * @param sizeY The height of the image.
 * @param image A shared pointer to the image message.
 * @return A shared pointer to a BoundingBox message indicating the detected ball's location.
 */
const relbot_interfaces::msg::BoundingBox::SharedPtr BallDetection::ball_detection_algorithm
    (int sizeX, int sizeY, const sensor_msgs::msg::Image::SharedPtr image) 
{
    // Init the bounding box for the ball detection
    auto bounding_box_msg = std::make_unique<relbot_interfaces::msg::BoundingBox>(); 

    // Init the pixel detected as part of the ball into a vector
    std::vector<int> ball_pixels_x;
    std::vector<int> ball_pixels_y;

    // Iterate through the image and identify ball pixels
    for (int x = 0; x < sizeX; x++)
    {
        for (int y = 0; y < sizeY; y++)
        {
            // Get the pixel colors for each of the channels
            std::tuple<int, int, int> pixel_channels = image_functions::getPixelChannels(image, x, y);
            // Get the average brightness
            int pixel_brightness = image_functions::getPixelBrightness(image, x, y);

            // Convert the RGB space into HSV space for image segmentation
            std::tuple<float, float, float> HSV_color = RGBtoHSV(pixel_channels, pixel_brightness);

            // HSV components of the pixel
            float hue = std::get<0>(HSV_color);
            float saturation = std::get<1>(HSV_color);
            float value = std::get<2>(HSV_color);

            // Check if the pixel meets the criterion in HSV space
            if (hue >= LOWER_BOUND_HSV[0] && hue <= UPPER_BOUND_HSV[0] &&
                saturation >= LOWER_BOUND_HSV[1] && saturation <= UPPER_BOUND_HSV[1] &&
                value >= LOWER_BOUND_HSV[2] && value <= UPPER_BOUND_HSV[2])
            {
                ball_pixels_x.push_back(x);
                ball_pixels_y.push_back(y);
            }
        }
    }

    // If no bounding box is found then return an empty box
    if(ball_pixels_x.empty() || ball_pixels_y.empty())
    {
        bounding_box_msg->ball_found = false;
        return bounding_box_msg;
    }

    // If ball is found then

    // Calculate bounding box dimensions based on detected ball pixels
    int min_x = *std::min_element(ball_pixels_x.begin(), ball_pixels_x.end());
    int max_x = *std::max_element(ball_pixels_x.begin(), ball_pixels_x.end());
    int min_y = *std::min_element(ball_pixels_y.begin(), ball_pixels_y.end());
    int max_y = *std::max_element(ball_pixels_y.begin(), ball_pixels_y.end());

    int bb_width = max_x - min_x + 1;
    int bb_height = max_y - min_y + 1;

    // If the bounding box is too rectangular then ignored it 
    // Difference between width and height greater than 40% 
    if((float)bb_width/(float)bb_height >= 1.40 || (float)bb_width/(float)bb_height <= 0.6)
    {
        bounding_box_msg->ball_found = false;
        return bounding_box_msg;
    }

    // If bounding box is good enough then return it
    // Fill in bounding box message data
    bounding_box_msg->ball_found = true;
    bounding_box_msg->center_point_x = (min_x + max_x + 1) / 2;
    bounding_box_msg->center_point_y = (min_y + max_y + 1) / 2;
    bounding_box_msg->width = bb_width;
    bounding_box_msg->height = bb_height;

    return bounding_box_msg;
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
