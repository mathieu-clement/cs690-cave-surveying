#include "pclviewer.h"
#include "fileloader.h"
#include "xyzpoint.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  //QApplication a (argc, argv);
  //PCLViewer w;
  //w.show ();

    const char* filename = "/tmp/input.pcd";
    FileLoader loader(filename);

    std::vector<XYZPoint>* points = loader.getPoints();
    std::vector<XYZPoint>::iterator it;
    for (it = points->begin() ;  it != points->end() ; it++) {
        XYZPoint point  = *it;
        printf("%f, %f, %f\n", point.x, point.y, point.z);
    }

    return 0;

  //return a.exec ();
}
