#include "params.h"

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
