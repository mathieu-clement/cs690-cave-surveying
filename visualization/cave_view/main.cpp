#include "pclviewer.h"
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
    return pclviewer_app(argc, argv);
}
