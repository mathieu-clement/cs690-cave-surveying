#include "pclviewer.h"
#include "build/ui_pclviewer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>


#include <QColorDialog>
#include <QFileDialog>
#include <QProgressDialog>

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
  viewer->setShowFPS(false);
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect buttons
  connect (ui->loadFileButton, SIGNAL(clicked()), this, SLOT(loadFileButtonPressed()));
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

    QProgressDialog progress("Loading file...", "Cancel", 0, 6, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.show();
    progress.raise();
    progress.activateWindow();

    // 0

    progress.setValue(0);
    if (progress.wasCanceled()) return;

    pcl::io::loadPCDFile(filename, *cloud);

    // 1

    progress.setValue(1);
    progress.setLabelText("Applying Moving Least Squares...");
    if (progress.wasCanceled()) return;

    pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ> mls;
    mls.setInputCloud(cloud);
    mls.setSearchRadius(10);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(5);
    mls.setUpsamplingStepSize(4);

    // 2

    progress.setValue(2);
    progress.setLabelText("Smoothing point cloud...");
    if (progress.wasCanceled()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ>());
    mls.process(*cloud_smoothed);

    // 3

    progress.setValue(3);
    progress.setLabelText("Computing normals...");
    if (progress.wasCanceled()) return;

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(cloud_smoothed);
    ne.setRadiusSearch(10);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_smoothed, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    for (size_t i = 0 ; i < cloud_normals->size() ; ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    // 4

    progress.setValue(4);
    progress.setLabelText("Concatenating points and normals...");
    if (progress.wasCanceled()) return;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

    // 5

    progress.setValue(5);
    progress.setLabelText("Computing mesh using Poisson...");
    if (progress.wasCanceled()) return;

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    // 6

    progress.setValue(6);

    viewer->removeAllPointClouds();
    viewer->addPointCloud (cloud_smoothed, "cloud_smoothed");
    viewer->addPolygonMesh(mesh);
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
