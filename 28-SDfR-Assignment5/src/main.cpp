//==============================================================
// Filename    : 
// Authors     : 
// Group       :
// License     :  N.A. or opensource license like LGPL
// Description : 
//==============================================================

#include <chrono>
#include <cmath>
#include <iostream>
#include <set>
#include <thread>
#include <vector>
#include <typeinfo>

#include <SDL2/SDL.h>

#include "UI.h"
#include "Blob.h"
#include "DrawContourScanning.h"
#include "DrawContourScanningThreaded.h"
#include "DrawContourMarching.h"
#include "DrawContourMarchingBetter.h"

const float thresholds[1] = {0.03}; // threshold the potential of a point needs to match

void printHelp() {
    std::cout << "The following keys can be pressed to perform an action:\n";
    std::cout << "Escape | Exit the program.\n";
    std::cout << "Space  | (Un)pause moving the blobs.\n";
    std::cout << "S      | Run the Scanning algorithm.\n";
    std::cout << "T      | Run the Threaded Scanning algorithm.\n";
    std::cout << "M      | Run the Marching algorithm.\n";
    std::cout << "B      | Run the Better Marching algorithm.\n";
} // end printHelp

int main(int /*argc*/, char ** /*argv*/)
{
    using namespace std::chrono;

    printHelp();

    UI gui(1000, 600);
    Blob blob;

    unsigned long frameCounter = 0;

    // define different algorithm objects
    DrawContourScanning drawContourScanning(gui, blob);
    DrawContourScanningThreaded drawContourScanningThreaded(gui, blob); 
    DrawContourMarching drawContourMarching(gui, blob);
    DrawContourMarchingBetter drawContourMarchingBetter(gui, blob);
    
    // set an algorithm object to the reference variable
    std::cout << "\nScanning algorithm loaded by default\n";
    DrawContour* drawContour = &drawContourScanning;

    bool pause = false;
    bool quit = false;
    while (!quit) {

        // pause the movement of the blob
        if (!pause) {
            blob.update(frameCounter++ / 2.0);
        }

        gui.clear(); // clear the ui
        high_resolution_clock::time_point t1 = high_resolution_clock::now(); // take first time point
        for (auto t : thresholds) {
            drawContour->drawContour(t); // draw contour for each threshold in a buffer (in the background)
        }
        high_resolution_clock::time_point t2 = high_resolution_clock::now(); // take second time point
        gui.present(); // show buffer
        std::cout << "\rComputation time: " << duration_cast<milliseconds>(t2 - t1).count() << " ms"; // output elapsed time in milliseconds in the console

        // handle the input
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            // quit button
            if (event.type == SDL_QUIT) {
                quit = true;
            }
            // all keydown events
            if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    std::cout << "\n\nExit program...";
                    quit = true;
                    break;
                case SDLK_m:
                    std::cout << "\n\nMarching algorithm now running.\n";
                    drawContour = &drawContourMarching;

                    break;
                case SDLK_b:
                    std::cout << "\n\nBetter marching algorithm now running.\n";
                    drawContour = &drawContourMarchingBetter;

                    break;
                case SDLK_s:
                    std::cout << "\n\nScanning algorithm now running.\n";
                    drawContour = &drawContourScanning;

                    break;
                case SDLK_t:
                    std::cout << "\n\nThreaded scanning algorithm now running.\n";
                    drawContour = &drawContourScanningThreaded;

                    break;
                case SDLK_SPACE:
                    pause = !pause;
                    std::cout << "\n\nMoving of blobs is now "<<(pause?"paused":"unpaused")<<"\n";
                    break;
                default:
                    break;
                }
            }
        }
    }

    return 0;
} // end main