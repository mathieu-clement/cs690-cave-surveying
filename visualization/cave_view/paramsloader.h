#ifndef PARAMSLOADER_H
#define PARAMSLOADER_H

#include "params.h"

#include <string>
#include <QString>

class ParamsLoader
{
public:
    ParamsLoader(std::string pcdFilepath);
    ParamsLoader(const char* pcdFilepath);

    bool exists();
    Params read();
    void write(Params params);

private:
    QString jsonFilepath;

    QString
    makeJsonFilepath(std::string pcdFilepath);

    std::string
    basename(std::string filename);
};

#endif // PARAMSLOADER_H
