#include "pclviewer.h"
#include "fileloader.h"
#include "xyzpoint.h"
#include "build/ui_pclviewer.h"

#include <QFileDialog>

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("Cave Viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));

  // Connect load file button
  connect (ui->loadFileButton, SIGNAL(clicked()), this, SLOT(loadFileButtonPressed()));
}

void
PCLViewer::loadFileButtonPressed()
{
    QString fileName = QFileDialog::getOpenFileName(
                this,
                "Load point cloud file",
                QDir::currentPath(),
                "PCF files (*.pcf)"
                );

    if (fileName.isNull()) return;

    loadPcdFile((char*) fileName.toStdString().c_str());
}

void
PCLViewer::loadPcdFile (char* filename)
{
    printf("Loading file: %s\n", filename);

    FileLoader loader(filename);

    std::vector<XYZPoint>* points = loader.getPoints();
    std::vector<XYZPoint>::iterator it;
    for (it = points->begin() ;  it != points->end() ; it++) {
        XYZPoint point  = *it;
        printf("%.3f %.3f %.3f\n", point.x, point.y, point.z);
    }

    // The default color
    red   = 128;
    green = 128;
    blue  = 128;

    // The number of points in the cloud
    cloud->points.resize (points->size());

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        XYZPoint p = points->at(i);
        cloud->points[i].x = p.x;
        cloud->points[i].y = p.y;
        cloud->points[i].z = p.z;

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
PCLViewer::RGBsliderReleased ()
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

void
PCLViewer::redSliderValueChanged (int value)
{
  red = value;
  printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
  RGBsliderReleased();
}

void
PCLViewer::greenSliderValueChanged (int value)
{
  green = value;
  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
  RGBsliderReleased();
}

void
PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
  RGBsliderReleased();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
