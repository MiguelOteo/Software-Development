// Point class definition.
#ifndef POINT_H
#define POINT_H

// Point class definition
class Point
{
public:
    const float x;
    const float y;

    Point();

    Point(float x, float y): x(x), y(y) {}; // constructor

    // < operator
    bool operator < (const Point &rhs) const
    {
        if (x == rhs.x)
            return y < rhs.y;
        else
            return x < rhs.x;
    }
}; // end class Point

#endif