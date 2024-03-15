// Blob class definition.
#ifndef BLOB_H
#define BLOB_H

#include <cmath> // C++ standard cmath class
#include "Point.h" // Point class declaration

// Blob class definition
class Blob
{
public:
    Blob() {}; // constructor

    float potential(Point p) const; // return potential for Point

    float potential(float x, float y) const; // return potential

    void update(float time); // update wells

private:
    const float scale = 100;
    float wells[2][2] = {{-100, -50}, {100, 50}};
}; // end class Blob

#endif