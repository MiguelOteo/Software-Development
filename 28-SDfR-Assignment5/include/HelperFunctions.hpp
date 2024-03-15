#include <iostream>
#include <vector>
#include "Blob.h"

#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

/**
 * @brief Find the first pixel which goes over the threshold
 * 
 * Find the first pixel using scalar potential values and scanning method.
 * The method compares the current value and checks if it goes over a threshold
 * if it does then check the previous to see if it is a boundary pixel, if so then 
 * returns it. Same principle when the previous values goes from over the threshold
 * to under the threshold
 *
 * @param threshold The threshold value for contour detection.
 * @return Point The pixel which is the first one to go over the threshold
 */
Point findInitialPixel
    (float threshold, int sizeX, int sizeY, Blob* blob);

/**
 * @brief Find multiple initial pixels which go over the threshold
 * 
 * Find the first pixel using scalar potential values and scanning method.
 * The method compares the current value and checks if it goes over a threshold
 * if it does then check the previous to see if it is a boundary pixel, if so then 
 * returns it. Same principle when the previous values goes from over the threshold
 * to under the threshold
 *
 * @param threshold The threshold value for contour detection.
 * @return std::vector<Point> Vector of initial pixels for each potential curve
 */
std::vector<Point> findInitialPixels
    (float threshold, int sizeX, int sizeY, Blob* blob, int& stepSize, int& maxPeaks);

#endif