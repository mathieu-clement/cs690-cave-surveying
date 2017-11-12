#include "poissonparamsdialog.h"
#include "build/ui_poissonparamsdialog.h"
#include "params.h"

PoissonParamsDialog::PoissonParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PoissonParamsDialog)
{
    ui->setupUi(this);
    setWindowTitle("Poisson");
}

PoissonParams
PoissonParamsDialog::getParams()
{
    unsigned int poissonDepth = ui->poissonDepthSpinBox->value();
    return (PoissonParams) { poissonDepth };
}

PoissonParamsDialog::~PoissonParamsDialog()
{
    delete ui;
}
