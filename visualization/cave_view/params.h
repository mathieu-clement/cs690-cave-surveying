#ifndef PARAMS_H
#define PARAMS_H

enum MeshAlgorithm {
    poisson,
    greedyProjectionTriangulation
};

typedef struct {
    unsigned int poissonDepth;
} PoissonParams;

typedef struct {
    unsigned int a;
} GreedyProjectionTriangulationParams;

typedef struct {
    union {
        PoissonParams poissonParams;
        GreedyProjectionTriangulationParams greedyProjectionTriangulationParams;
    };
} MeshParams;

typedef struct {
    bool mlsEnabled;
    double mlsSearchRadius;
    double mlsUpsamplingRadius;
    double mlsUpsamplingStepSize;
    double normalsSearchRadius;
    unsigned int normalsThreads;
    MeshAlgorithm meshAlgorithm;
    MeshParams meshParams;
} Params;

#endif // PARAMS_H
