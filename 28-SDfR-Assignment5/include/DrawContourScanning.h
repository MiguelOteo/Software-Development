//==============================================================
// Filename    : 
// Authors     : 
// Group       :
// License     :  N.A. or opensource license like LGPL
// Description : 
//==============================================================

// DrawContourScanning class derived from class
// DrawContour.
#ifndef DRAWCONTOURSCANNING_H
#define DRAWCONTOURSCANNING_H

#include "DrawContour.h" // DrawContour class declaration

// DrawContourScanning class definition
class DrawContourScanning : public DrawContour
{
public:
    // constructor
    DrawContourScanning(UI &ui, Blob &blob): DrawContour(ui, blob) {};

    // draw contour
    virtual void drawContour(float threshold);
}; // end class DrawContourScanning

#endif