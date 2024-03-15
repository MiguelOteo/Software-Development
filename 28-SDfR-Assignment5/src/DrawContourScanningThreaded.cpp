//==============================================================
// Filename    : 
// Authors     : 
// Group       :
// License     :  N.A. or opensource license like LGPL
// Description : 
//==============================================================

#include "DrawContourScanningThreaded.h" 
#include <thread>
#include <vector>
#include <mutex>
#include <iostream>

// Define mutex for synchronization
std::mutex mutex;

/**
 * @brief Thread function for contour detection.
 * 
 * This function calculates scalar potential at each pixel position within a specified range
 * and detects contour crossings based on a threshold value.
 * 
 * @param threadIndex The index of the current thread.
 * @param numThreads The total number of threads.
 * @param sizeX The size of the region along the x-axis.
 * @param sizeY The size of the region along the y-axis.
 * @param threshold The threshold value for contour detection.
 * @param blob Pointer to the Blob object containing potential calculation method.
 * @param ui Pointer to the UI object for drawing.
 */
void threadFunction
    (int threadIndex, int numThreads, int sizeX, int sizeY, float threshold, Blob* blob, UI* ui) 
{
    // Calculate the range of pixels this thread will handle 
    // The area is a rectangle which takes the full width of the panel
    // and a certain width which depends on the number of threads of the CPU
    int startY = (sizeY * threadIndex / numThreads) - sizeY/2;
    int endY = (sizeY * (threadIndex + 1) / numThreads) - sizeY/2;

    // Iterate through each pixel in the assigned range
    for (int x = -sizeX/2; x < sizeX/2; x++)
    {
        for (int y = startY; y < endY; y++)
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
                // Synchronize access to drawPixel
                {
                    std::lock_guard<std::mutex> lock(mutex); // Lock
                    ui->drawPixel(x, y);
                } // Unlock
            }

            // Check if the scalar value crosses the threshold for y-coordinate
            if ((scalarValue > threshold && scalarPreviousY <= threshold) || 
                (scalarValue <= threshold && scalarPreviousY > threshold))
            {
                // Synchronize access to drawPixel
                {
                    std::lock_guard<std::mutex> lock(mutex); // Lock
                    ui->drawPixel(x, y);
                } // Unlock
            }
        }   
    }
}

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
void DrawContourScanningThreaded::drawContour(float threshold) 
{
    // Get the size of the drawing area from the UI
    int sizeX = ui->sizeX; 
    int sizeY = ui->sizeY;

    // Define the number of threads you want to use
    const int numThreads = std::thread::hardware_concurrency();
    
    // Define a vector to hold thread objects
    std::vector<std::thread> threads(numThreads);

    // Launch threads
    for (int threadNum = 0; threadNum < numThreads; ++threadNum) {
        threads[threadNum] = std::thread
            (threadFunction, threadNum, numThreads, sizeX, sizeY, threshold, blob, ui);
    }

    // Join threads
    for (auto& thread : threads) {
        thread.join();
    }
}
