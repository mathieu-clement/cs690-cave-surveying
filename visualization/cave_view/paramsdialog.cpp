#include "paramsdialog.h"
#include "poissonparamsdialog.h"
#include "build/ui_paramsdialog.h"
#include "greedyprojectiontriangulationparamsdialog.h"
#include "marchingcubesparamsdialog.h"

#include <iostream>

ParamsDialog::ParamsDialog(QWidget *parent, Params* previousParams) :
    QDialog(parent),
    ui(new Ui::ParamsDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("Mesh Parameters");
    connect (ui->configureMeshButton, SIGNAL(clicked()), this, SLOT(configureMesh()));
    connect (ui->meshAlgorithmComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(activateMeshAlgorithm(int)));

    if (previousParams != nullptr) {
        loadParams(previousParams);
    } else {
        activateMeshAlgorithm(0);
    }
}

Params
ParamsDialog::getParams()
{
    bool mlsEnabled = ui->mlsEnableCheckBox->isChecked();
    double mlsSearchRadius = ui->mlsSearchRadiusSpinBox->value();
    unsigned int mlsPolynomialOrder = ui->mlsPolynomialOrderSpinBox->value();
    double mlsUpsamplingRadius = ui->mlsUpsamplingRadiusSpinBox->value();
    double mlsUpsamplingStepSize = ui->mlsUpsamplingStepSizeSpinBox->value();
    double normalsSearchRadius = ui->normalsSearchRadiusSpinBox->value();
    unsigned int normalsThreads = ui->normalsThreadsSpinBox->value();

    Params params = {
                        mlsEnabled, mlsSearchRadius, mlsPolynomialOrder,
                        mlsUpsamplingRadius, mlsUpsamplingStepSize,
                        normalsSearchRadius, normalsThreads,
                        meshAlgorithm, meshParams
                    };
    return params;
}

void
ParamsDialog::activateMeshAlgorithm(int index)
{
    activeMeshAlgorithmIndex = index;

    // Default values

    meshAlgorithm = getSelectedMeshAlgorithm();
    switch (meshAlgorithm) {
        case poisson:
            {
                meshParams.poissonParams = (PoissonParams) { 9 };
                break;
            }
        case greedyProjectionTriangulation:
            {
                meshParams.greedyProjectionTriangulationParams = (GreedyProjectionTriangulationParams) {
                        200, 15.0, 3.0
                };
                break;
            }
        case marchingCubes:
            {
                meshParams.marchingCubesParams = (MarchingCubesParams) {
                        0.0f, 100, 100, 100, 0.2f
                };
                break;
            }
    }
}

void
ParamsDialog::configureMesh()
{
    MeshAlgorithm algo = getSelectedMeshAlgorithm();
    switch (algo) {
        case poisson:
        {
            PoissonParamsDialog dialog(this, &(meshParams.poissonParams));
            dialog.exec();
            meshParams.poissonParams = dialog.getParams();
            break;
        }

        case greedyProjectionTriangulation:
        {
            GreedyProjectionTriangulationParamsDialog dialog(this,
                &(meshParams.greedyProjectionTriangulationParams));
            dialog.exec();
            meshParams.greedyProjectionTriangulationParams = dialog.getParams();
            break;
        }

        case marchingCubes:
        {
            MarchingCubesParamsDialog dialog(this, &(meshParams.marchingCubesParams));
            dialog.exec();
            meshParams.marchingCubesParams = dialog.getParams();
            break;
        }

        default:
            std::cerr << "Cannot configure mesh. Algorithm '" << algo << "' unknown." << std::endl;
            throw algo;
    }
}

MeshAlgorithm
ParamsDialog::getSelectedMeshAlgorithm()
{
    switch(activeMeshAlgorithmIndex) {
        case 0:
            return poisson;
        case 1:
            return greedyProjectionTriangulation;
        case 2:
            return marchingCubes;
    }
    std::cerr << "Mesh algorithm of index '" << activeMeshAlgorithmIndex << "' unknown." << std::endl;
    throw activeMeshAlgorithmIndex;
}

void
ParamsDialog::loadParams(Params *params)
{
    ui->mlsEnableCheckBox->setChecked(params->mlsEnabled);
    ui->mlsSearchRadiusSpinBox->setValue(params->mlsSearchRadius);
    ui->mlsUpsamplingStepSizeSpinBox->setValue(params->mlsUpsamplingStepSize);
    ui->mlsUpsamplingRadiusSpinBox->setValue(params->mlsUpsamplingRadius);
    ui->mlsPolynomialOrderSpinBox->setValue(params->mlsPolynomialOrder);

    ui->normalsSearchRadiusSpinBox->setValue(params->normalsSearchRadius);
    ui->normalsThreadsSpinBox->setValue(params->normalsThreads);

    previousMeshAlgorithm = params->meshAlgorithm;
    unsigned int index = -1;
    switch (params->meshAlgorithm) {
        case poisson: index = 0; break;
        case greedyProjectionTriangulation: index = 1; break;
        case marchingCubes: index = 2; break;
        default:
            {
                std::cerr << "Mesh algorithm '" << params->meshAlgorithm << "' unknown." << std::endl;
                throw params->meshAlgorithm;
            }
    }

    activeMeshAlgorithmIndex = index;
    meshAlgorithm = getSelectedMeshAlgorithm();
    ui->meshAlgorithmComboBox->setCurrentIndex(index);

    meshParams = params->meshParams;
}

ParamsDialog::~ParamsDialog()
{
    delete ui;
}
