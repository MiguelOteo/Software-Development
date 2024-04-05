#ifndef BOUNDING_BOX_DETECTION_HPP
#define BOUNDING_BOX_DETECTION_HPP

#include "sensor_msgs/msg/image.hpp"
#include "relbot_interfaces/msg/bounding_box.hpp"
#include "image_functions_sdfr/image_functions.hpp"
#include "../include/ros2_ball_detection/BoundingBoxDetection.hpp"

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
void ball_pixels_detection
    (int thread_index, int num_threads, int sizeX, int sizeY, 
     const sensor_msgs::msg::Image::SharedPtr image, 
     std::pair<std::vector<int>, std::vector<int>>& ball_pixels);

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
    (std::pair<std::vector<int>, std::vector<int>>& ball_pixels);

#endif /* BOUNDING_BOX_DETECTION_HPP */