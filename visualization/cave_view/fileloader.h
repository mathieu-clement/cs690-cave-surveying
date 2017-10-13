#ifndef FILELOADER_H
#define FILELOADER_H

#include "xyzpoint.h"
#include <string>
#include <vector>

class FileLoader
{
public:
    FileLoader(const char* filename);

    std::vector<XYZPoint>* getPoints();

private:
    std::vector<XYZPoint> list;
};

#endif // FILELOADER_H
