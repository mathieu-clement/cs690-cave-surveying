#include "params.h"
#include <cstring>

const char* MeshAlgorithmCString(MeshAlgorithm e)
{
    switch (e) {
        case poisson:
            return "poisson";
        case greedyProjectionTriangulation:
            return "greedyProjectionTriangulation";
        case marchingCubes:
            return "marchingCubes";
    }
    throw e;
}

MeshAlgorithm CStringMeshAlgorithm(const char* s)
{
    if (strcmp(s, "poisson") == 0) return poisson;
    if (strcmp(s, "greedyProjectionTriangulation") == 0) return greedyProjectionTriangulation;
    if (strcmp(s, "marchingCubes") == 0) return marchingCubes;
    throw s;
}

bool smoothingParamsChanged(Params *p1, Params *p2)
{
    return p1->mlsEnabled != p2->mlsEnabled ||
        p1->mlsPolynomialOrder != p2->mlsPolynomialOrder ||
        p1->mlsSearchRadius != p2->mlsSearchRadius ||
        p1->mlsUpsamplingRadius != p2->mlsUpsamplingRadius ||
        p1->mlsUpsamplingStepSize != p2->mlsUpsamplingStepSize;
}

bool normalsParamsChanged(Params *p1, Params *p2)
{
    return p1->normalsSearchRadius != p2->normalsSearchRadius;
}
