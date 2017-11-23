#ifndef PARAMSDIALOG_H
#define PARAMSDIALOG_H

#include <QDialog>

#include "params.h"

namespace Ui {
    class ParamsDialog;
}

class ParamsDialog : public QDialog {
Q_OBJECT

public:
    explicit ParamsDialog(QWidget *parent = 0, Params *previousParams = nullptr);

    ~ParamsDialog();

    Params getParams();

public Q_SLOTS:

    void activateMeshAlgorithm(int index);

    void configureMesh();

    void radioButtonToggled(bool state);

protected:
    MeshAlgorithm meshAlgorithm;
    MeshParams meshParams;
    int activeMeshAlgorithmIndex;

    MeshAlgorithm getSelectedMeshAlgorithm();

private:
    Ui::ParamsDialog *ui;
    MeshAlgorithm previousMeshAlgorithm;
    bool removeOutliers = true;

    void loadParams(Params *params);
};

#endif // PARAMSDIALOG_H
