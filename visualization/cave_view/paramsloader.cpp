#include "paramsloader.h"

#include "json.hpp"
#include <fstream>
#include <QDir>
#include <QFileInfo>
#include <QString>

#ifndef JADD
#define JADD(js, para, name) js[#name] = para.name
#endif

#ifndef JGET
#define JGET(js, para, name) para.name = js[#name]
#endif

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
    json j;
    std::ifstream(jsonFilepath.toUtf8().constData()) >> j;

    Params params;

    JGET(j, params, mlsEnabled);
    JGET(j, params, mlsSearchRadius);
    JGET(j, params, mlsPolynomialOrder);
    JGET(j, params, mlsUpsamplingRadius);
    JGET(j, params, mlsUpsamplingStepSize);
    JGET(j, params, normalsSearchRadius);
    JGET(j, params, normalsThreads);
    std::string meshAlgorithmStr = j["meshAlgorithm"];
    params.meshAlgorithm = CStringMeshAlgorithm(meshAlgorithmStr.c_str());

    switch (params.meshAlgorithm) {
        case poisson:
            params.meshParams.poissonParams.poissonDepth = j["meshParams"]["poissonDepth"];
            break;

        case greedyProjectionTriangulation:
        {
            GreedyProjectionTriangulationParams p = params.meshParams.greedyProjectionTriangulationParams;
            p.maxNearestNeighbors = j["meshParams"]["maxNearestNeighbors"];
            p.searchRadius = j["meshParams"]["searchRadius"];
            p.mu = j["meshParams"]["mu"];
            break;
        }

        case marchingCubes:
        {
            MarchingCubesParams p = params.meshParams.marchingCubesParams;
            p.isoLevel = j["meshParams"]["isoLevel"];
            p.gridResolutionX = j["meshParams"]["gridResolutionX"];
            p.gridResolutionY = j["meshParams"]["gridResolutionY"];
            p.gridResolutionZ = j["meshParams"]["gridResolutionZ"];
            p.gridExtensionPercentage = j["meshParams"]["gridExtensionPercentage"];
            break;
        }

        default:
            throw params.meshAlgorithm;
    }

    return params;
}

void
ParamsLoader::write(Params params)
{
    json j;
    JADD(j, params, mlsEnabled);
    JADD(j, params, mlsSearchRadius);
    JADD(j, params, mlsPolynomialOrder);
    JADD(j, params, mlsUpsamplingRadius);
    JADD(j, params, mlsUpsamplingStepSize);
    JADD(j, params, normalsSearchRadius);
    JADD(j, params, normalsThreads);
    j["meshAlgorithm"] = MeshAlgorithmCString(params.meshAlgorithm);

    switch (params.meshAlgorithm) {
        case poisson:
            JADD(j["meshParams"], params.meshParams.poissonParams, poissonDepth);
            break;

        case greedyProjectionTriangulation:
            JADD(j["meshParams"], params.meshParams.greedyProjectionTriangulationParams, maxNearestNeighbors);
            JADD(j["meshParams"], params.meshParams.greedyProjectionTriangulationParams, searchRadius);
            JADD(j["meshParams"], params.meshParams.greedyProjectionTriangulationParams, mu);
            break;

        case marchingCubes:
            JADD(j["meshParams"], params.meshParams.marchingCubesParams, isoLevel);
            JADD(j["meshParams"], params.meshParams.marchingCubesParams, gridResolutionX);
            JADD(j["meshParams"], params.meshParams.marchingCubesParams, gridResolutionY);
            JADD(j["meshParams"], params.meshParams.marchingCubesParams, gridResolutionZ);
            JADD(j["meshParams"], params.meshParams.marchingCubesParams, gridExtensionPercentage);
            break;
    }

    std::ofstream o(jsonFilepath.toUtf8().constData());
    o << std::setw(4) << j << std::endl;
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
