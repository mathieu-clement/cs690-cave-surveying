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
