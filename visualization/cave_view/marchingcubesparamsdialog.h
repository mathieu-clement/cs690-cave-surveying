#ifndef MARCHINGCUBESPARAMSDIALOG_H
#define MARCHINGCUBESPARAMSDIALOG_H

#include <QDialog>
#include "params.h"

namespace Ui {
class MarchingCubesParamsDialog;
}

class MarchingCubesParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MarchingCubesParamsDialog(QWidget *parent = 0);

    MarchingCubesParams getParams();

    ~MarchingCubesParamsDialog();

private:
    Ui::MarchingCubesParamsDialog *ui;
};

#endif // MARCHINGCUBESPARAMSDIALOG_H
