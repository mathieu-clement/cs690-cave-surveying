#include "mlsparamsdialog.h"
#include "build/ui_mlsparamsdialog.h"

MLSParamsDialog::MLSParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MLSParamsDialog)
{
    ui->setupUi(this);
    this->setWindowTitle("Mesh Parameters");
}

MLSParams
MLSParamsDialog::getMlsParams()
{
    bool mlsEnabled = ui->mlsEnableCheckBox->isChecked();
    double mlsSearchRadius = ui->mlsSearchRadiusSpinBox->value();
    double mlsUpsamplingRadius = ui->mlsUpsamplingRadiusSpinBox->value();
    double mlsUpsamplingStepSize = ui->mlsUpsamplingStepSizeSpinBox->value();
    double normalsSearchRadius = ui->normalsSearchRadiusSpinBox->value();
    unsigned int normalsThreads = ui->normalsThreadsSpinBox->value();
    unsigned int poissonDepth = ui->poissonDepthSpinBox->value();
    MLSParams params = {
                        mlsEnabled, mlsSearchRadius, mlsUpsamplingRadius, mlsUpsamplingStepSize,
                        normalsSearchRadius, normalsThreads,
                        poissonDepth
                       };
    return params;
}

MLSParamsDialog::~MLSParamsDialog()
{
    delete ui;
}
