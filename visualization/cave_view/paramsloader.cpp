#include "paramsloader.h"

#include "json.hpp"
#include "params.h"
#include <fstream>
#include <iostream>
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

ParamsLoader::ParamsLoader(std::string pcdFilepath) {
    jsonFilepath = makeJsonFilepath(pcdFilepath);
}

ParamsLoader::ParamsLoader(const char *pcdFilepath) : ParamsLoader::ParamsLoader(std::string(pcdFilepath)) {

}

bool
ParamsLoader::exists() {
    return QFileInfo(jsonFilepath).exists();
}

Params
ParamsLoader::read() {
    // Malformed JSON files WILL crash the program
    // Older JSON format MIGHT crash the program
    // TODO Add a "version" property

    json j;
    std::ifstream(jsonFilepath.toUtf8().constData()) >> j;

    Params params;

    JGET(j, params, removeOutliers);
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
            params.meshParams.poissonParams = (PoissonParams) {j["meshParams"]["poissonDepth"]};
            break;

        case greedyProjectionTriangulation: {
            params.meshParams.greedyProjectionTriangulationParams = (GreedyProjectionTriangulationParams) {
                    j["meshParams"]["maxNearestNeighbors"],
                    j["meshParams"]["searchRadius"],
                    j["meshParams"]["mu"]
            };
            break;
        }

        case marchingCubes: {
            params.meshParams.marchingCubesParams = (MarchingCubesParams) {
                    (float) j["meshParams"]["isoLevel"],
                    j["meshParams"]["gridResolutionX"],
                    j["meshParams"]["gridResolutionY"],
                    j["meshParams"]["gridResolutionZ"],
                    (float) j["meshParams"]["gridExtensionPercentage"]
            };
            break;
        }

        case noMesh: {
            params = {};
            break;
        }

        default:
            throw params.meshAlgorithm;
    }

    return params;
}

void
ParamsLoader::write(Params params) {
    json j;
    JADD(j, params, removeOutliers);
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

        case noMesh:
            break;
    }

    std::ofstream o(jsonFilepath.toUtf8().constData());
    o << std::setw(4) << j << std::endl;
}

QString
ParamsLoader::makeJsonFilepath(std::string pcdFilepath) {
    QFileInfo pcdFi(QString::fromStdString(pcdFilepath));
    std::string pcdFilename = pcdFi.fileName().toStdString();
    std::string bname = basename(pcdFilename);
    std::string jsonFilename = bname + ".json";
    return pcdFi.dir().absoluteFilePath(QString::fromStdString(jsonFilename));
}

std::string
ParamsLoader::basename(std::string filename) {
    return filename.substr(0, filename.find_last_of('.'));
}
