#ifndef MLSPARAMSDIALOG_H
#define MLSPARAMSDIALOG_H

#include <QDialog>

#include "mlsparams.h"

namespace Ui {
class MLSParamsDialog;
}

class MLSParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MLSParamsDialog(QWidget *parent = 0);
    ~MLSParamsDialog();

    MLSParams
    getMlsParams();

private:
    Ui::MLSParamsDialog *ui;
};

#endif // MLSPARAMSDIALOG_H
