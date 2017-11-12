#ifndef PARAMS_H
#define PARAMS_H

enum MeshAlgorithm { poisson };

typedef struct {
    double normalsSearchRadius;
    unsigned int normalsThreads;
    unsigned int poissonDepth;
} PoissonParams;

typedef struct {
    //union {
        PoissonParams poissonParams;
    //}
} MeshParams;

typedef struct {
    bool mlsEnabled;
    double mlsSearchRadius;
    double mlsUpsamplingRadius;
    double mlsUpsamplingStepSize;
    MeshAlgorithm meshAlgorithm;
    MeshParams meshParams;
} Params;

#endif // PARAMS_H
