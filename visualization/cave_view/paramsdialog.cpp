#include "paramsdialog.h"
#include "build/ui_paramsdialog.h"

ParamsDialog::ParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ParamsDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("Mesh Parameters");
}

Params
ParamsDialog::getParams()
{
    bool mlsEnabled = ui->mlsEnableCheckBox->isChecked();
    double mlsSearchRadius = ui->mlsSearchRadiusSpinBox->value();
    double mlsUpsamplingRadius = ui->mlsUpsamplingRadiusSpinBox->value();
    double mlsUpsamplingStepSize = ui->mlsUpsamplingStepSizeSpinBox->value();
    double normalsSearchRadius = ui->normalsSearchRadiusSpinBox->value();
    unsigned int normalsThreads = ui->normalsThreadsSpinBox->value();
    unsigned int poissonDepth = ui->poissonDepthSpinBox->value();
    MeshAlgorithm meshAlgorithm = poisson;
    PoissonParams poissonParams = {
        normalsSearchRadius, normalsThreads, poissonDepth
    };
    Params params = {
                        mlsEnabled, mlsSearchRadius, mlsUpsamplingRadius, mlsUpsamplingStepSize,
                        meshAlgorithm, poissonParams = poissonParams
                    };
    return params;
}

ParamsDialog::~ParamsDialog()
{
    delete ui;
}
