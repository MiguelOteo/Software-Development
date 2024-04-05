#ifndef RGB_TO_HSV_HPP
#define RGB_TO_HSV_HPP

#include <tuple>

/**
 * @brief Converts an RGB color represented as integers to its corresponding HSV representation.
 * 
 * This function takes an RGB color represented as integers in the range [0, 255] and 
 * converts it to its corresponding HSV (Hue, Saturation, Value) representation. The pixel 
 * brightness is also considered in the conversion process.
 *
 * @param RGB_color A tuple representing the RGB color values (R, G, B) where each 
 *  component is an integer in the range [0, 255].
 * @param pixel_brightness An integer representing the brightness of the pixel in 
 *  the range [0, 255].
 * 
 * @return A tuple representing the HSV color values (H, S, V) where H is the hue in degrees (0-360), S is the 
 *         saturation (0-1), and V is the value (brightness) normalized to the range [0, 1].
 */
std::tuple<float, float, float> RGBtoHSV(std::tuple<int, int, int>& RGB_color, int& pixel_brightness);

#endif /* RGB_TO_HSV_HPP */