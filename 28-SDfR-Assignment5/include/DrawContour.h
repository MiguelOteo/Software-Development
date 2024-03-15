// DrawContour class definition.
#ifndef DRAWCONTOUR_H
#define DRAWCONTOUR_H

#include "UI.h" // UI class declaration
#include "Blob.h" // Blob class declaration

#include <iostream>

// DrawContour class definition
class DrawContour
{
public:
    DrawContour(UI &ui, Blob &blob): ui(&ui), blob(&blob) {} // constructor

    virtual void drawContour(float threshold) = 0; // draw contour

protected:
    UI*   ui;
    Blob* blob;
}; // end class DrawContour

#endif