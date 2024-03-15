#include <math.h>
#include <iostream>
#include <vector>
#include "HelperFunctions.hpp"
#include "Blob.h"
#include "Point.h"

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
Point findInitialPixel(float threshold, int sizeX, int sizeY, Blob* blob)
{
    // Iterate through each pixel in the drawing area
    for (int x = -sizeX/2; x < sizeX/2; x++)
    {
        for (int y = -sizeY/2; y < sizeY/2; y++)
        {
            // Calculate scalar potential at the current position
            double scalarValue = blob->potential(x, y);

            // Calculate scalar potential at the previous position (x - 1)
            double scalarPreviousX = (x > 0) ? blob->potential(x - 1, y) : scalarValue;

            // Calculate scalar potential at the previous position (y - 1)
            double scalarPreviousY = (y > 0) ? blob->potential(x, y - 1) : scalarValue;

            // Check if the scalar value crosses the threshold for x-coordinate
            if ((scalarValue > threshold && scalarPreviousX <= threshold) || 
                (scalarValue <= threshold && scalarPreviousX > threshold))
            {
                // return the pixel
                Point initPixel = Point(x, y);
                return initPixel;
            }

            // Check if the scalar value crosses the threshold for y-coordinate
            if ((scalarValue > threshold && scalarPreviousY <= threshold) || 
                (scalarValue <= threshold && scalarPreviousY > threshold))
            {
                // return the pixel
                Point initPixel = Point(x, y);
                return initPixel;
            }
        }
    }
    Point initPixel = Point(-1000, -1000);
    return initPixel;
}

/**
 * @brief Find multiple initial pixels which go over the threshold
 * 
 * Find the first pixels using scalar potential values and scanning method.
 * The method compares the current value and checks if it goes over a threshold
 * if it does then check the previous to see if it is a boundary pixel, if so then 
 * returns it. Same principle when the previous values goes from over the threshold
 * to under the threshold
 *
 * @param threshold The threshold value for contour detection.
 * @return std::vector<Point> Vector of initial pixels for each potential curve
 */
std::vector<Point> findInitialPixels(float threshold, int sizeX, int sizeY, Blob* blob, int& stepSize, int& maxPeaks)
{
    // Declare the vector of the pixels
    std::vector<Point> initPixels;

    // Variable to know the number of peaks scanned
    int numPeaks = 0;

    // Variable to know if the previous column is empty or not
    bool prevColumnIsEmpty = true;

    // Iterate through each pixel in the drawing area
    for (int x = -sizeX/2; x < sizeX/2; x += stepSize)
    {
        std::vector<Point> initPixelsColumn;

        for (int y = -sizeY/2; y < sizeY/2; y++)
        {
            // Calculate scalar potential at the current position
            double scalarValue = blob->potential(x, y);

            // Calculate scalar potential at the previous position (x - 1)
            double scalarPreviousX = (x > 0) ? blob->potential(x - 1, y) : scalarValue;

            // Calculate scalar potential at the previous position (y - 1)
            double scalarPreviousY = (y > 0) ? blob->potential(x, y - 1) : scalarValue;

            // Check if the scalar value crosses the threshold for x-coordinate
            if ((scalarValue > threshold && scalarPreviousX <= threshold) || 
                (scalarValue <= threshold && scalarPreviousX > threshold))
            {
                // Return the pixel
                Point initPixel = Point(x, y);
                initPixelsColumn.push_back(initPixel);
                continue;
            }

            // Check if the scalar value crosses the threshold for y-coordinate
            if ((scalarValue > threshold && scalarPreviousY <= threshold) || 
                (scalarValue <= threshold && scalarPreviousY > threshold))
            {
                // Return the pixel
                Point initPixel = Point(x, y);
                initPixelsColumn.push_back(initPixel);
            }
        }

        // If columns is empty then skip it and set the is empty to true
        if(initPixelsColumn.empty())
        {
            prevColumnIsEmpty = true;
            continue;
        }

        // If column is not empty and prev column is true then add one to the number of peaks
        if(prevColumnIsEmpty == true)
        {
            prevColumnIsEmpty = false;
            numPeaks++;
        }

        // Add the initial pixels of the column to the total list
        for(auto& pixel: initPixelsColumn)
        {
            initPixels.push_back(pixel);
        }

        // Skip earlier if the number of peaks detected is greater 
        // or equal to the max number of peaks
        if(numPeaks >= maxPeaks)
        {
            return initPixels;
        }
    }
    return initPixels;
}
