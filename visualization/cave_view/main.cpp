#include "pclviewer.h"
#include "params.h"
#include "paramsloader.h"
#include <QApplication>
#include <QMainWindow>

int pclviewer_app(int argc, char *argv[])
{
    QApplication a (argc, argv);
    PCLViewer w;
    w.show ();
    return a.exec();
}

int main (int argc, char *argv[])
{
    //return pclviewer_app(argc, argv);

    ParamsLoader loader = "/tmp/test.pcd";
    MeshParams meshParams;
    meshParams.marchingCubesParams = (MarchingCubesParams) { 1.0f, 2, 3, 4, 5.0f };
    Params params = (Params) {
            true,
            1.0,
            2,
            3.0,
            4.0,
            5.0,
            6,
            marchingCubes,
            meshParams
    };
    loader.write(params);

    return 0;
}
