#ifndef POISSONPARAMSDIALOG_H
#define POISSONPARAMSDIALOG_H

#include <QDialog>
#include "params.h"

namespace Ui {
class PoissonParamsDialog;
}

class PoissonParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit PoissonParamsDialog(QWidget *parent = 0, PoissonParams* previousParams = nullptr);
    ~PoissonParamsDialog();

    PoissonParams getParams();

private:
    Ui::PoissonParamsDialog *ui;
};

#endif // POISSONPARAMSDIALOG_H
