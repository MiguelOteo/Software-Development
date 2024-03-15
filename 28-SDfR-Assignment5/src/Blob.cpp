// Blob class definition.

#include <cmath> // C++ standard cmath class
#include "Blob.h"

// return potential for Point
float Blob::potential(Point p) const { 
    return potential(p.x, p.y); 
}

// return potential
float Blob::potential(float x, float y) const {
    return scale *
        (1.0 / (pow(x - wells[0][0], 2) + pow(y - wells[0][1], 2)) +
         1.0 / (pow(x - wells[1][0], 2) + pow(y - wells[1][1], 2)));
}

// update wells
void Blob::update(float time) {
    wells[0][0] = -100 + 30 * sin(time / 7);
    wells[0][1] = -50 * cos(time / 5);
    wells[1][0] = -wells[0][0];
    wells[1][1] = -wells[0][1];
}