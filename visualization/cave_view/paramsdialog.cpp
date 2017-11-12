#include "paramsdialog.h"
#include "poissonparamsdialog.h"
#include "build/ui_paramsdialog.h"

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
    double mlsUpsamplingRadius = ui->mlsUpsamplingRadiusSpinBox->value();
    double mlsUpsamplingStepSize = ui->mlsUpsamplingStepSizeSpinBox->value();

    Params params = {
                        mlsEnabled, mlsSearchRadius, mlsUpsamplingRadius, mlsUpsamplingStepSize,
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
            meshParams.poissonParams = (PoissonParams) { 10, 8, 9 };
            break;
            }
        case greedyProjectionTriangulation:
            {
            meshParams.greedyProjectionTriangulationParams = (GreedyProjectionTriangulationParams) {
                    0
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
                PoissonParamsDialog dialog(this);
                dialog.exec();
                meshParams = (MeshParams) { dialog.getParams() };
                break;
            }
        case greedyProjectionTriangulation:
            int a = 1;
            break;
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
    }
    throw activeMeshAlgorithmIndex;
}

ParamsDialog::~ParamsDialog()
{
    delete ui;
}
