//==============================================================
// Filename    : 
// Authors     : 
// Group       :
// License     :  N.A. or opensource license like LGPL
// Description : 
//==============================================================

#include "DrawContourScanning.h" 
#include "HelperFunctions.hpp"
#include <iostream>

/**
 * @brief Draw contour lines
 * 
 * Draw contour lines based on scalar potential values using scanning method.
 * The method compares the current value and checks if it goes over a threshold
 * if it does then check the previous to see if it is a boundary pixel, if so then 
 * paint it white. Same principle when the previous values goes from over the threshold
 * to under the threshold
 *
 * @param threshold The threshold value for contour detection.
 */
void DrawContourScanning::drawContour(float threshold) 
{
    // Get the size of the drawing area from the UI
    int sizeX = ui->sizeX; 
    int sizeY = ui->sizeY;

    // Iterate through each pixel in the drawing area
    for (int x = -sizeX/2; x < sizeX/2; x++)
    {
        for (int y = -sizeY/2; y < sizeY/2; y++)
        {
            // Calculate scalar potential at the current position
            double scalarValue = blob->potential(x, y);
            
            // Calculate scalar potential at the previous position (x - 1)
            double scalarPreviousX = blob->potential(x - 1, y);

            // Calculate scalar potential at the previous position (y - 1)
            double scalarPreviousY = blob->potential(x, y - 1);

            // Check if the scalar value crosses the threshold for x-coordinate
            if ((scalarValue > threshold && scalarPreviousX <= threshold) || 
                (scalarValue <= threshold && scalarPreviousX > threshold))
            {
                // Paint the pixel white if it is a boundary pixel
                ui->drawPixel(x, y);
            }

            // Check if the scalar value crosses the threshold for y-coordinate
            if ((scalarValue > threshold && scalarPreviousY <= threshold) || 
                (scalarValue <= threshold && scalarPreviousY > threshold))
            {
                // Paint the pixel white if it is a boundary pixel
                ui->drawPixel(x, y);
            }
        }   
    }
}