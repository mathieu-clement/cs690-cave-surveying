#ifndef XYZPOINT_H
#define XYZPOINT_H

class XYZPoint
{
public:
    XYZPoint(float x, float y, float     z);
    float getX();
    float getY();
    float getZ();

private:
    float x;
    float y;
    float z;
};

#endif // XYZPOINT_H
