#ifndef PARAMS_H
#define PARAMS_H

enum MeshAlgorithm {
    poisson,
    greedyProjectionTriangulation,
    marchingCubes
};

typedef struct {
    unsigned int poissonDepth;
} PoissonParams;

typedef struct {
    unsigned int maxNearestNeighbors;
    double searchRadius;
    double mu;
} GreedyProjectionTriangulationParams;

typedef struct {
    unsigned int a;
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

#endif // PARAMS_H
