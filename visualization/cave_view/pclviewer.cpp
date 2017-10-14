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

  // Connect checkboxes
  connect (ui->showPointsCheckbox, SIGNAL(toggled(bool)), this, SLOT(showPointsCheckBoxToggled(bool)));
  connect (ui->showMeshCheckbox, SIGNAL(toggled(bool)), this, SLOT(showMeshCheckBoxToggled(bool)));

  // Controls inactive until file is loaded
  disableUi();
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

    loadPcdFile(fileName.toStdString());
}

void
PCLViewer::loadPcdFile (std::string filename)
{
    std::cout << "Loading file: " << filename << std::endl;

    disableUi();

    QProgressDialog progress("Loading file...", "Cancel", 0, 6, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.show();

    // Progress: 0

    progress.setValue(0);
    if (progress.wasCanceled()) return;

    // Copied / inspired from:
    // http://www.pointclouds.org/assets/icra2012/surface.pdf

    pcl::io::loadPCDFile(filename.c_str(), *cloud);

    // Progress: 1

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

    // Progress: 2

    progress.setValue(2);
    progress.setLabelText("Smoothing point cloud...");
    if (progress.wasCanceled()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr *pCloud_smoothed =
            new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    this->cloud_smoothed = pCloud_smoothed;
    mls.process(**pCloud_smoothed);

    // Progress: 3

    progress.setValue(3);
    progress.setLabelText("Computing normals...");
    if (progress.wasCanceled()) return;

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(8);
    ne.setInputCloud(*pCloud_smoothed);
    ne.setRadiusSearch(10);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(**pCloud_smoothed, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
    ne.compute(*cloud_normals);

    for (size_t i = 0 ; i < cloud_normals->size() ; ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }

    // Progress: 4

    progress.setValue(4);
    progress.setLabelText("Concatenating points and normals...");
    if (progress.wasCanceled()) return;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(**pCloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

    // Progress: 5

    progress.setValue(5);
    progress.setLabelText("Computing mesh using Poisson...");
    if (progress.wasCanceled()) return;

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    pcl::PolygonMesh *pMesh = new pcl::PolygonMesh;
    this->mesh = pMesh;
    poisson.reconstruct(*pMesh);

    // Progress: END
    progress.setValue(6);

    viewer->removeAllPointClouds();
    viewer->addPointCloud (*pCloud_smoothed, "cloud_smoothed");
    viewer->addPolygonMesh(*pMesh, "mesh");
    viewer->resetCamera();
    ui->qvtkWidget->update();

    enableUi();
    ui->showPointsCheckbox->setChecked(true);
    ui->showMeshCheckbox->setChecked(true);

    this->raise();
    this->activateWindow();
}

void
PCLViewer::showPointsCheckBoxToggled(bool checked)
{
    if(checked) {
        viewer->addPointCloud(*this->cloud_smoothed, "cloud_smoothed");
    } else {
        viewer->removePointCloud("cloud_smoothed");
    }
    ui->qvtkWidget->update();
}

void
PCLViewer::showMeshCheckBoxToggled(bool checked)
{
    if(checked) {
        viewer->addPolygonMesh(*this->mesh, "mesh");
    } else {
        viewer->removePolygonMesh("mesh");
    }
    ui->qvtkWidget->update();
}

void
PCLViewer::disableUi()
{
    setUiEnabled(false);
}

void
PCLViewer::enableUi()
{
    setUiEnabled(true);
}

void
PCLViewer::setUiEnabled(bool enabled)
{
    ui->showPointsCheckbox->setEnabled(enabled);
    ui->showMeshCheckbox->setEnabled(enabled);
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
