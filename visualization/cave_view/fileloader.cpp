#include "fileloader.h"
#include "xyzpoint.h"
#include <string>
#include <vector>
#include <sstream>
#include <fstream>

FileLoader::FileLoader(const char* filename)
{
    std::ifstream infile(filename);
    float x, y, z;
    while (infile >> x >> y >> z) {
        XYZPoint point(x, y, z);
        list.push_back(point);
    }
}

std::vector<XYZPoint>* FileLoader::getPoints()
{
    return &list;
}
