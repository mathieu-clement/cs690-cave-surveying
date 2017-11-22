#include "poissonparamsdialog.h"
#include "build/ui_poissonparamsdialog.h"
#include "params.h"

PoissonParamsDialog::PoissonParamsDialog(QWidget *parent, PoissonParams *previousParams) :
        QDialog(parent),
        ui(new Ui::PoissonParamsDialog) {
    ui->setupUi(this);
    setWindowTitle("Poisson");

    if (previousParams != nullptr) {
        ui->poissonDepthSpinBox->setValue(previousParams->poissonDepth);
    }
}

PoissonParams
PoissonParamsDialog::getParams() {
    unsigned int poissonDepth = ui->poissonDepthSpinBox->value();
    return (PoissonParams) {poissonDepth};
}

PoissonParamsDialog::~PoissonParamsDialog() {
    delete ui;
}
