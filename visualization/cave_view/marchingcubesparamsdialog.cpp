#include "marchingcubesparamsdialog.h"
#include "build/ui_marchingcubesparamsdialog.h"
#include "params.h"

MarchingCubesParamsDialog::MarchingCubesParamsDialog(QWidget *parent, MarchingCubesParams *previousParams) :
        QDialog(parent),
        ui(new Ui::MarchingCubesParamsDialog) {
    ui->setupUi(this);
    setWindowTitle("Marching Cubes");

    if (previousParams != nullptr) {
        ui->isoLevelSpinBox->setValue(previousParams->isoLevel);
        ui->gridResolutionXSpinBox->setValue(previousParams->gridResolutionX);
        ui->gridResolutionYSpinBox->setValue(previousParams->gridResolutionY);
        ui->gridResolutionZSpinBox->setValue(previousParams->gridResolutionZ);
        ui->gridExtensionPercentageSpinBox->setValue(previousParams->gridExtensionPercentage);
    }
}

MarchingCubesParams
MarchingCubesParamsDialog::getParams() {
    float isoLevel = (float) ui->isoLevelSpinBox->value();
    unsigned int gridResolutionX = ui->gridResolutionXSpinBox->value();
    unsigned int gridResolutionY = ui->gridResolutionYSpinBox->value();
    unsigned int gridResolutionZ = ui->gridResolutionZSpinBox->value();
    float gridExtensionPercentage = (float) ui->gridExtensionPercentageSpinBox->value();

    return (MarchingCubesParams) {
            isoLevel, gridResolutionX, gridResolutionY, gridResolutionZ, gridExtensionPercentage
    };
}

MarchingCubesParamsDialog::~MarchingCubesParamsDialog() {
    delete ui;
}
