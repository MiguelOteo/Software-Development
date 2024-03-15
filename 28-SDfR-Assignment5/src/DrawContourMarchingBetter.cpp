//==============================================================
// Filename    : 
// Authors     : 
// Group       :
// License     :  N.A. or opensource license like LGPL
// Description : 
//==============================================================

#include "DrawContourMarchingBetter.h" 
#include "HelperFunctions.hpp"
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include "Point.h"

// Step size in the x axis to avoid excesive computations
int STEP_SIZE_X = 2;
// Number of peaks in the potential function to stop execution of code earlier
int MAX_PEAKS = 2;

// draw contour
void DrawContourMarchingBetter::drawContour(float threshold) {

    // Get the size of the drawing area from the UI
    int sizeX = ui->sizeX; 
    int sizeY = ui->sizeY;

    // Create a set for the visited pixels
    std::set<Point> visitedList;

    // Create a vector for the worklist
    std::vector<Point> workList;

    // Find the initial pixel using the existing scanning method
    std::vector<Point> initialPixels = findInitialPixels(threshold, sizeX, sizeY, blob, STEP_SIZE_X, MAX_PEAKS);

    // If no blob was found then decrease the step size
    if(initialPixels.empty() && STEP_SIZE_X > 1)
    {
        STEP_SIZE_X = STEP_SIZE_X - 1;
    }
        
    // Add the initial pixel to the worklist and the visited list
    for (auto& initialPixel: initialPixels)
    {
        workList.push_back(initialPixel);       
    }
    
    while (!workList.empty()) 
    {
        // Take and remove the pixel from the worklist
        Point currentPixel = workList.back();
        workList.pop_back();

        // Skip if the pixel is already visited
        if (visitedList.count(currentPixel) > 0)
        {
            continue;
        }

        // Convert pixel coordinates to array indices
        int currentX = currentPixel.x;
        int currentY = currentPixel.y;

        // Calculate scalar potential at the corners of the pixel
        double scalarCorners[4];
        scalarCorners[0] = blob->potential(currentX - 0.5, currentY - 0.5);
        scalarCorners[1] = blob->potential(currentX - 0.5, currentY + 0.5);
        scalarCorners[2] = blob->potential(currentX + 0.5, currentY - 0.5);
        scalarCorners[3] = blob->potential(currentX + 0.5, currentY + 0.5);

        bool crosses[4];
        for (int i = 0; i < 4; ++i) 
        {
            crosses[i] = (scalarCorners[i] > threshold) != (scalarCorners[(i + 1) % 4] > threshold);
        }

        // Check if the P = v curve crosses any line segments
        if (crosses[0] || crosses[1] || crosses[2] || crosses[3]) 
        {
            // Draw the pixel if the curve crosses
            ui->drawPixel(currentX, currentY);

            // Add all 8 neighbors of the pixel to the worklist
            for (int x = -1; x <= 1; x++) 
            {
                for (int y = -1; y <= 1; y++) 
                {
                    if (x == 0 && y == 0) 
                    {
                        continue;
                    }
                    Point pixel = Point(currentX + x, currentY + y);
                    workList.push_back(pixel);
                }   
            }
        }
        
        // Add the current pixel to the visited list
        visitedList.insert(currentPixel);
    }
}