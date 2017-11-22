#ifndef PARAMS_H
#define PARAMS_H

enum MeshAlgorithm {
    poisson,
    greedyProjectionTriangulation,
    marchingCubes,
    noMesh
};

const char *MeshAlgorithmCString(MeshAlgorithm e);

MeshAlgorithm CStringMeshAlgorithm(const char *s);

typedef struct {
    unsigned int poissonDepth;
} PoissonParams;

typedef struct {
    unsigned int maxNearestNeighbors;
    double searchRadius;
    double mu;
} GreedyProjectionTriangulationParams;

typedef struct {
    float isoLevel;
    unsigned int gridResolutionX;
    unsigned int gridResolutionY;
    unsigned int gridResolutionZ;
    float gridExtensionPercentage;
} MarchingCubesParams;

typedef struct {
    union {
        PoissonParams poissonParams;
        GreedyProjectionTriangulationParams greedyProjectionTriangulationParams;
        MarchingCubesParams marchingCubesParams;
    };
} MeshParams;

typedef struct {
    bool mlsEnabled;
    double mlsSearchRadius;
    unsigned int mlsPolynomialOrder;
    double mlsUpsamplingRadius;
    double mlsUpsamplingStepSize;
    double normalsSearchRadius;
    unsigned int normalsThreads;
    MeshAlgorithm meshAlgorithm;
    MeshParams meshParams;
} Params;

bool smoothingParamsChanged(Params *p1, Params *p2);

bool normalsParamsChanged(Params *p1, Params *p2);

#endif // PARAMS_H
