#include "xyzpoint.h"

XYZPoint::XYZPoint(float x, float y, float z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

float XYZPoint::getX()
{
    return this->x;
}

float XYZPoint::getY()
{
    return this->y;
}

float XYZPoint::getZ()
{
    return this->z;
}
