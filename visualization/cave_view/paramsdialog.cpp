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
            meshParams = (MeshParams) {
                (PoissonParams) { 10, 8, 9 }
            };
    }
}

void
ParamsDialog::configureMesh()
{
    MeshAlgorithm algo = getSelectedMeshAlgorithm();
    switch (algo) {
        case poisson:
            PoissonParamsDialog dialog(this);
            dialog.exec();
            meshParams = (MeshParams) { dialog.getParams() };
            break;
    }
}

MeshAlgorithm
ParamsDialog::getSelectedMeshAlgorithm()
{
    switch(activeMeshAlgorithmIndex) {
        case 0:
            return poisson;
    }
    throw activeMeshAlgorithmIndex;
}

ParamsDialog::~ParamsDialog()
{
    delete ui;
}
