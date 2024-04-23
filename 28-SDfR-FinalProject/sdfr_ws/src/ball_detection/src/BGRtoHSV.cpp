#include <tuple>
#include <cmath>
#include <algorithm>
#include "../include/ros2_ball_detection/BGRtoHSV.hpp"
#include "image_functions_sdfr/image_functions.hpp"

/**
 * @brief Converts an BGR color represented as integers to its corresponding HSV representation.
 * 
 * This function takes an BGR color represented as integers in the range [0, 255] and 
 * converts it to its corresponding HSV (Hue, Saturation, Value) representation. The pixel 
 * brightness is also considered in the conversion process.
 *
 * @param BGR_color A tuple representing the BGR color values (B, G, R) where each 
 *  component is an integer in the range [0, 255].
 * @param pixel_brightness An integer representing the brightness of the pixel in 
 *  the range [0, 255].
 * 
 * @return A tuple representing the HSV color values (H, S, V) where H is the hue in degrees (0-360), S is the 
 *         saturation (0-1), and V is the value (brightness) normalized to the range [0, 1].
 */
std::tuple<float, float, float> BGRtoHSV(std::tuple<int, int, int>& BGR_color, int& pixel_brightness) 
{
    // Convert BGR values to the range [0, 1]
    float R = std::get<2>(BGR_color) / 255.0f;
    float G = std::get<1>(BGR_color) / 255.0f;
    float B = std::get<0>(BGR_color) / 255.0f;
    float norm_brigthness = pixel_brightness / 255.0f;

    // Calculate maximum, minimum, and delta values
    float fMax = std::max(std::max(R, G), B);
    float fMin = std::min(std::min(R, G), B);
    float fDelta = fMax - fMin;

    // Initialize variables for HSV components
    float fH = 0.0f;    
    float fS = 0.0f;
    float fV = norm_brigthness;

    // If delta is greater than 0, calculate Hue and Saturation
    if (fDelta > 0) 
    {
        if (fMax == R) 
        {
            // Hue calculation when the maximum is red
            fH = 60 * std::fmod(((G - B) / fDelta), 6);
        } 
        else if (fMax == G) 
        {
            // Hue calculation when the maximum is green
            fH = 60 * (((B - R) / fDelta) + 2);
        } 
        else if (fMax == B) 
        {
            // Hue calculation when the maximum is blue
            fH = 60 * (((R - G) / fDelta) + 4);
        }

        // Saturation calculation
        if (fMax > 0) 
        {
            fS = fDelta / fMax;
        }
    }

    // Adjust Hue if it's negative
    if (fH < 0) 
    {
        fH += 360;
    }

    // Return the HSV color tuple
    return std::make_tuple(fH, fS, fV);
}
