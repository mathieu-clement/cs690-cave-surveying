#include <iostream>
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
    meshParams.poissonParams = (PoissonParams) {
            9
    };
    Params params = (Params) {
            true,
            1.0,
            2,
            3.0,
            4.0,
            5.0,
            6,
            poisson,
            meshParams
    };
    loader.write(params);

    Params params2 = loader.read();

    ParamsLoader loader2 = "/tmp/test2.pcd";
    loader2.write(params2);

    return 0;
}
