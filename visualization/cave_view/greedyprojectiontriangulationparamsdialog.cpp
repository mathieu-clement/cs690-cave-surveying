#include "greedyprojectiontriangulationparamsdialog.h"
#include "build/ui_greedyprojectiontriangulationparamsdialog.h"
#include "params.h"

GreedyProjectionTriangulationParamsDialog::GreedyProjectionTriangulationParamsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GreedyProjectionTriangulationParamsDialog)
{
    ui->setupUi(this);
    setWindowTitle("Greedy Projection Triangulation Parameters");
}

GreedyProjectionTriangulationParams
GreedyProjectionTriangulationParamsDialog::getParams()
{
    unsigned int maxNearestNeighbors = ui->maxNearestNeighborsSpinBox->value();
    double searchRadius = ui->searchRadiusSpinBox->value();
    double mu = ui->muSpinBox->value();
    return (GreedyProjectionTriangulationParams) { maxNearestNeighbors, searchRadius, mu };
}

GreedyProjectionTriangulationParamsDialog::~GreedyProjectionTriangulationParamsDialog()
{
    delete ui;
}
