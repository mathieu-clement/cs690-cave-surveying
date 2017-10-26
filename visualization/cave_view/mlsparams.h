#ifndef MLSPARAMS_H
#define MLSPARAMS_H

typedef struct {
    bool mlsEnabled;
    double mlsSearchRadius;
    double mlsUpsamplingRadius;
    double mlsUpsamplingStepSize;
    double normalsSearchRadius;
    unsigned int normalsThreads;
    unsigned int poissonDepth;
} MLSParams;

#endif // MLSPARAMS_H
