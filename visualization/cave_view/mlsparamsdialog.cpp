#include "mlsparamsdialog.h"
#include "build/ui_mlsparamsdialog.h"

MLSParamsDialog::MLSParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MLSParamsDialog)
{
    ui->setupUi(this);
}

MLSParams
MLSParamsDialog::getMlsParams()
{
    unsigned int searchRadius = ui->searchRadiusSpinBox->value();
    unsigned int upsamplingRadius = ui->upsamplingRadiusSpinBox->value();
    unsigned int upsamplingStepSize = ui->upsamplingStepSizeSpinBox->value();
    MLSParams params = {searchRadius, upsamplingRadius, upsamplingStepSize};
    return params;
}

MLSParamsDialog::~MLSParamsDialog()
{
    delete ui;
}
