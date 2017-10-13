#include "pclviewer.h"
#include "build/ui_pclviewer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>

#include <QColorDialog>
#include <QFileDialog>

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("Cave Viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);

  // The default color
  red   = 128;
  green = 128;
  blue  = 128;

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect buttons
  connect (ui->loadFileButton, SIGNAL(clicked()), this, SLOT(loadFileButtonPressed()));
  connect (ui->changeColorButton, SIGNAL(clicked()), this, SLOT(changeColorButtonPressed()));
}

void
PCLViewer::loadFileButtonPressed()
{
    QString fileName = QFileDialog::getOpenFileName(
                this,
                "Load point cloud data file",
                QDir::currentPath(),
                "PCD files (*.pcd)"
                );

    if (fileName.isNull()) return;

    loadPcdFile((char*) fileName.toStdString().c_str());
}

void
PCLViewer::loadPcdFile (char* filename)
{
    printf("Loading file: %s\n", filename);

    pcl::io::loadPCDFile(filename, *cloud);

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }

    viewer->removeAllPointClouds();
    viewer->addPointCloud (cloud, "cloud");
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

void
PCLViewer::changeColorButtonPressed()
{
    QColor color = QColorDialog::getColor(Qt::white, this);
    if (!color.isValid()) return;
    red = color.red();
    green = color.green();
    blue = color.blue();
    colorChanged();
}

void
PCLViewer::colorChanged ()
{
  // Set the new color
  for (size_t i = 0; i < cloud->size (); i++)
  {
    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }
  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
