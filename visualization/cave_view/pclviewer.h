#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QKeyEvent>
#include <QMainWindow>
#include <QProgressDialog>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "params.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
    class PCLViewer;
}

class PCLViewer : public QMainWindow {
Q_OBJECT

public:
    explicit PCLViewer(QWidget *parent = 0);

    ~PCLViewer();

public Q_SLOTS:

    void
    loadFileButtonPressed();

    void
    showPointsCheckBoxToggled(bool checked);

    void
    showNormalsCheckboxToggled(bool checked);

    void
    showMeshCheckBoxToggled(bool checked);

    void
    changeParameters();

    void
    resetCamera();

    void
    setBackgroundColorInt(int percents);

    void
    radioButtonToggled(bool state);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    PointCloudT::Ptr cloud;

    void
    loadPcdFile(std::string filename);

    void
    disableUi();

    void
    enableUi();

    void
    keyReleaseEvent(QKeyEvent *);

private:
    Ui::PCLViewer *ui;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr *cloud_smoothed;
    pcl::PointCloud<pcl::Normal>::Ptr *cloud_normals;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr *cloud_smoothed_normals;
    pcl::PolygonMesh *mesh;
    std::string lastFilename = "";

    void
    setUiEnabled(bool enabled);

    // Returns false if progress dialog cancelled
    bool
    updateProgress(int step, QString message, QProgressDialog *dialog);

    void
    applyPoisson(PoissonParams poissonParams);

    void
    applyGreedyProjectionTriangulation(GreedyProjectionTriangulationParams params);

    void
    applyMarchingCubes(MarchingCubesParams params);

    float
    distance(pcl::PointCloud<pcl::PointXYZRGB>::iterator it);

    float
    distance(pcl::PointXYZRGB p);

    void
    removeOutliers();

    void
    addPointCloud();

    void
    setBackgroundColor(float grayLevel);

    void
    setBackgroundBlack();

    void
    colorize();

    void
    showMeshAsWireframe();

    void
    showMeshAsPoints();

    void
    showMeshAsSurface();

    void
    showNormals();

    void
    hideNormals();
};

#endif // PCLVIEWER_H
