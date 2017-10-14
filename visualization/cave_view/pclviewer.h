#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "mlsparams.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public Q_SLOTS:
  void
  loadFileButtonPressed ();

  void
  showPointsCheckBoxToggled (bool checked);

  void
  showMeshCheckBoxToggled (bool checked);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;

  void
  loadPcdFile (std::string filename);

  void
  disableUi();

  void
  enableUi();

  MLSParams
  getMlsParams();

private:
  Ui::PCLViewer *ui;

  pcl::PointCloud<pcl::PointXYZ>::Ptr *cloud_smoothed;
  pcl::PolygonMesh *mesh;

  void
  setUiEnabled (bool enabled);
};

#endif // PCLVIEWER_H
