#ifndef PARAMS_H
#define PARAMS_H

typedef struct {
    bool mlsEnabled;
    double mlsSearchRadius;
    double mlsUpsamplingRadius;
    double mlsUpsamplingStepSize;
    double normalsSearchRadius;
    unsigned int normalsThreads;
    unsigned int poissonDepth;
} Params;

#endif // PARAMS_H
