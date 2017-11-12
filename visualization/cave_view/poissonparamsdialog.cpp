#include "poissonparamsdialog.h"
#include "build/ui_poissonparamsdialog.h"
#include "params.h"

PoissonParamsDialog::PoissonParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PoissonParamsDialog)
{
    ui->setupUi(this);
    setWindowTitle("Normals Estimation and Poisson Parameters");
}

PoissonParams
PoissonParamsDialog::getParams()
{
    double normalsSearchRadius = ui->normalsSearchRadiusSpinBox->value();
    unsigned int normalsThreads = ui->normalsThreadsSpinBox->value();
    unsigned int poissonDepth = ui->poissonDepthSpinBox->value();
    return (PoissonParams) { normalsSearchRadius, normalsThreads, poissonDepth };
}

PoissonParamsDialog::~PoissonParamsDialog()
{
    delete ui;
}
