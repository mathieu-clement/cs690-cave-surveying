#include "paramsdialog.h"
#include "poissonparamsdialog.h"
#include "build/ui_paramsdialog.h"
#include "greedyprojectiontriangulationparamsdialog.h"

ParamsDialog::ParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ParamsDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("Mesh Parameters");
    connect (ui->configureMeshButton, SIGNAL(clicked()), this, SLOT(configureMesh()));
    connect (ui->meshAlgorithmComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(activateMeshAlgorithm(int)));

    activateMeshAlgorithm(0);
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
                meshParams.marchingCubesParams = (MarchingCubesParams) { 0 };
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
                PoissonParamsDialog dialog(this);
                dialog.exec();
                meshParams.poissonParams = dialog.getParams();
                break;
            }
        case greedyProjectionTriangulation:
            {
                GreedyProjectionTriangulationParamsDialog dialog(this);
                dialog.exec();
                meshParams.greedyProjectionTriangulationParams = dialog.getParams();
                break;
            }
            case marchingCubes:
            {
                int a = 1;
                break;
            }
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
    throw activeMeshAlgorithmIndex;
}

ParamsDialog::~ParamsDialog()
{
    delete ui;
}
