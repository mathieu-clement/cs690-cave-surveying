#ifndef GREEDYPROJECTIONTRIANGULATIONPARAMSDIALOG_H
#define GREEDYPROJECTIONTRIANGULATIONPARAMSDIALOG_H

#include <QDialog>
#include "params.h"

namespace Ui {
class GreedyProjectionTriangulationParamsDialog;
}

class GreedyProjectionTriangulationParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GreedyProjectionTriangulationParamsDialog(
            QWidget *parent = 0, GreedyProjectionTriangulationParams* previousParams = nullptr);

    GreedyProjectionTriangulationParams getParams();

    ~GreedyProjectionTriangulationParamsDialog();

private:
    Ui::GreedyProjectionTriangulationParamsDialog *ui;
};

#endif // GREEDYPROJECTIONTRIANGULATIONPARAMSDIALOG_H
