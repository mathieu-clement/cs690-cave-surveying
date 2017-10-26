#include "pclviewer.h"
#include "build/ui_pclviewer.h"
#include "mlsparams.h"
#include "mlsparamsdialog.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/features/normal_3d_omp.h>

#include <QApplication>
#include <QColorDialog>
#include <QFileDialog>
#include <QProgressDialog>
#include <QSound>

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
  ui->filenameLabel->setText(QString::Null());

  this->raise();
  this->activateWindow();
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

    QProgressDialog progress("Loading file...", "Cancel", 0, 5, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.show();

    // Progress: 0

    progress.setValue(0);
    if (progress.wasCanceled()) { return; }

    // Copied / inspired from:
    // http://www.pointclouds.org/assets/icra2012/surface.pdf

    pcl::io::loadPCDFile(filename.c_str(), *cloud);

    // Progress: 1

    if(!updateProgress(1, "Smoothing...", &progress)) { return; }

    MLSParams mlsParams = getMlsParams();

    QSound sound("/Users/mathieuclement/Downloads/jeopardy_think.wav");
    sound.play();

    pcl::PointCloud<pcl::PointXYZ>::Ptr *pCloud_smoothed;

    if (mlsParams.mlsEnabled) {
        pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ> mls;
        mls.setInputCloud(cloud);
        mls.setSearchRadius(mlsParams.mlsSearchRadius);
        mls.setPolynomialFit(true);
        mls.setPolynomialOrder(2);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(mlsParams.mlsUpsamplingRadius);
        mls.setUpsamplingStepSize(mlsParams.mlsUpsamplingStepSize);

        pCloud_smoothed = new pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        this->cloud_smoothed = pCloud_smoothed;
        mls.process(**pCloud_smoothed);
    } else {
        pCloud_smoothed = new pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud);
    }

    this->cloud_smoothed = pCloud_smoothed;

    // Progress: 2

    if(!updateProgress(2, "Estimating normals...", &progress)) { sound.stop(); return; }

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads(mlsParams.normalsThreads);
    ne.setInputCloud(*pCloud_smoothed);
    ne.setRadiusSearch(mlsParams.normalsSearchRadius);
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

    // Progress: 3

    if(!updateProgress(3, "Concatenating points and normals...", &progress)) { sound.stop(); return; }

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(**pCloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

    // Progress: 4

    if(!updateProgress(4, "Computing mesh using Poisson...", &progress)) { sound.stop(); return; }

    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(mlsParams.poissonDepth);
    poisson.setInputCloud(cloud_smoothed_normals);
    pcl::PolygonMesh *pMesh = new pcl::PolygonMesh;
    this->mesh = pMesh;
    poisson.reconstruct(*pMesh);

    // Progress: END
    progress.setValue(5);

    viewer->removeAllPointClouds();
    viewer->addPointCloud (*pCloud_smoothed, "cloud_smoothed");
    viewer->addPolygonMesh(*pMesh, "mesh");
    viewer->resetCamera();
    ui->qvtkWidget->update();

    enableUi();
    ui->showPointsCheckbox->setChecked(true);
    ui->showMeshCheckbox->setChecked(true);

    QFileInfo fi(QString::fromStdString(filename));
    ui->filenameLabel->setText(fi.fileName());

    sound.stop();

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
  
bool
PCLViewer::updateProgress (int step, QString message, QProgressDialog *dialog)
{
    std::cout << "Progress (" << step << "): " << message.toAscii().constData() << std::endl;
    if (dialog->wasCanceled()) return false;
    dialog->setValue(step);
    dialog->setLabelText(message);
    QApplication::processEvents();
    return true;
}

MLSParams
PCLViewer::getMlsParams()
{
    MLSParamsDialog dialog(this);
    dialog.setModal(true);
    dialog.exec();
    return dialog.getMlsParams();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
