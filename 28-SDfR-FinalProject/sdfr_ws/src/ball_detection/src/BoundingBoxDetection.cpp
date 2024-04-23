#include <mutex>
#include <vector>
#include <iostream>
#include <thread>
#include "sensor_msgs/msg/image.hpp"
#include "relbot_interfaces/msg/bounding_box.hpp"
#include "image_functions_sdfr/image_functions.hpp"
#include "../include/ros2_ball_detection/BGRtoHSV.hpp"
#include "../include/ros2_ball_detection/BoundingBoxDetection.hpp"

// Define a mutex for synchronization
std::mutex mutex;

// Define the upper and lower bound in the HSV space
// Hue [0-360] Red around 0 | Green at 120 | Blue at 240
// Stauration [0-1]
// Value [0-1]
const float LOWER_BOUND_HSV[3] = {140, 0.3, 0.1};
const float UPPER_BOUND_HSV[3] = {170, 1.0, 0.9};

/**
 * @brief Detects pixels belonging to a ball within a specified image region.
 * 
 * This function performs ball pixel detection within a specified region of an image.
 * It analyzes the RGB and brightness values of each pixel, converts them to HSV space,
 * and checks if they fall within predefined HSV bounds for a ball.
 * 
 * @param thread_index The index of the current thread.
 * @param num_threads The total number of threads.
 * @param sizeX The width of the image.
 * @param sizeY The height of the image.
 * @param image A shared pointer to the image message.
 * @param ball_pixels A pair of vectors to store the x and y coordinates of 
 * detected ball pixels.
 */
void ball_pixels_detection
    (int thread_index, int num_threads, int sizeX, int sizeY, 
     const sensor_msgs::msg::Image::SharedPtr image, 
     std::pair<std::vector<int>, std::vector<int>>& ball_pixels)
{
    // Get the size of the drawing area from the webcam image
    int startY = (sizeY*thread_index/num_threads);
    int endY = (sizeY*(thread_index + 1)/num_threads);

    // Iterate through the image and identify ball pixels
    for (int x = 0; x < sizeX; x++)
    {
        // Iterate through the column which belongs to the area of the thread
        for (int y = startY; y < endY; y++)
        {
            // Get the pixel colors for each of the channels
            std::tuple<int, int, int> pixel_channels = image_functions::getPixelChannels(image, x, y);
            // Get the average brightness
            int pixel_brightness = image_functions::getPixelBrightness(image, x, y);

            // Convert the RGB space into HSV space for image segmentation
            std::tuple<float, float, float> HSV_color = BGRtoHSV(pixel_channels, pixel_brightness);

            // HSV components of the pixel
            float hue = std::get<0>(HSV_color);
            float saturation = std::get<1>(HSV_color);
            float value = std::get<2>(HSV_color);

            // Check if the pixel meets the criterion in HSV space
            if (hue >= LOWER_BOUND_HSV[0] && hue <= UPPER_BOUND_HSV[0] &&
                saturation >= LOWER_BOUND_HSV[1] && saturation <= UPPER_BOUND_HSV[1] &&
                value >= LOWER_BOUND_HSV[2] && value <= UPPER_BOUND_HSV[2])
            {
                // Synchronize access to the pixels vectors
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    // Add pixels x coordinate
                    ball_pixels.first.push_back(x);
                    // Add pixel y coordinate
                    ball_pixels.second.push_back(y);
                }
            }
        }
    }
}

/**
 * @brief Determines the bounding box of a detected ball from its pixel coordinates.
 * 
 * This function calculates the bounding box of a detected ball based on the pixel 
 * coordinates obtained from the ball pixel detection process. It checks if a valid 
 * bounding box can be formed from the detected pixels and returns a message indicating 
 * the ball's location within the image.
 * 
 * @param ball_pixels A pair of vectors containing the x and y coordinates of detected ball pixels.
 * @return A shared pointer to a BoundingBox message indicating the detected ball's location.
 */
const relbot_interfaces::msg::BoundingBox::SharedPtr get_bounding_box
    (std::pair<std::vector<int>, std::vector<int>>& ball_pixels) 
{
    // Init the bounding box for the ball detection
    auto bounding_box_msg = std::make_unique<relbot_interfaces::msg::BoundingBox>(); 

    // Init the pixel detected as part of the ball into a vector
    std::vector<int> ball_pixels_x = ball_pixels.first;
    std::vector<int> ball_pixels_y = ball_pixels.second;

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