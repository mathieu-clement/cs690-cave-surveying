#include "paramsloader.h"

#include "json.hpp"
#include <fstream>
#include <QDir>
#include <QFileInfo>
#include <QString>

using json = nlohmann::json;

ParamsLoader::ParamsLoader(std::string pcdFilepath)
{
    jsonFilepath = makeJsonFilepath(pcdFilepath);
}

ParamsLoader::ParamsLoader(const char* pcdFilepath) : ParamsLoader::ParamsLoader(std::string(pcdFilepath))
{

}

bool
ParamsLoader::exists()
{
    return QFileInfo(jsonFilepath).exists();
}

Params
ParamsLoader::read()
{
    return (Params) { };
}

void
ParamsLoader::write(Params params)
{
    json j;
    j["mls_enabled"] = params.mlsEnabled;

    std::ofstream o(jsonFilepath.toUtf8().constData());
    o << std::setw(4) << j << std::endl;
    o.close();
}

QString
ParamsLoader::makeJsonFilepath(std::string pcdFilepath)
{
    QFileInfo pcdFi(QString::fromStdString(pcdFilepath));
    std::string pcdFilename = pcdFi.fileName().toStdString();
    std::string bname = basename(pcdFilename);
    std::string jsonFilename = bname + ".json";
    return pcdFi.dir().absoluteFilePath(QString::fromStdString(jsonFilename));
}

std::string
ParamsLoader::basename(std::string filename)
{
    return filename.substr(0, filename.find_last_of("."));
}
