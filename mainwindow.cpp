#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "qtopencv.h"
#include "openglscene.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ocv = ui->tab;
    ogl = ui->tab_2;

    this->updatePrincipalPoint();

    connect(ocv,SIGNAL(autoCalib()),this,SLOT(updatePrincipalPoint()));

    connect(ocv->ui.pushButton_Quit,SIGNAL(clicked()),this,SLOT(close()));
    connect(ui->actionQuit,SIGNAL(triggered()),this,SLOT(close()));
    connect(ui->actionAbout_Qt4,SIGNAL(triggered()),qApp,SLOT(aboutQt()));
    connect(ui->actionLoad_Image,SIGNAL(triggered()),this,SLOT(loadImage()));
    connect(ui->actionLoad_Video,SIGNAL(triggered()),this,SLOT(loadVideo()));
    connect(ui->actionOpen_Camera,SIGNAL(triggered()),this,SLOT(loadCamera()));
    connect(ui->actionAbout,SIGNAL(triggered()),this,SLOT(aboutDlg()));
    connect(ui->actionShow_Auto_Calibration,SIGNAL(toggled(bool)),ocv,SLOT(toggleDrawCalib(bool)));
    connect(ui->actionShow_Feature_Extraction,SIGNAL(toggled(bool)),ocv,SLOT(toggleDrawFeatures(bool)));
    connect(ui->actionShow_3D_Estimation,SIGNAL(toggled(bool)),ocv,SLOT(toggleDrawPose(bool)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadImage()
{
    QString fname = QFileDialog::getOpenFileName(this,
                                                 "Select Image File",
                                                 "",
                                                 "Image Files (*.png *.jpg *.gif *.bmp *.tif)"
                                                 );
    if(fname.size() > 0)
        ocv->openImage(fname);
}

void MainWindow::loadVideo()
{
    QString fname = QFileDialog::getOpenFileName(this,
                                                 "Select Video File",
                                                 "",
                                                 "Video Files (*.mpg *.mpeg *.avi)"
                                                 );
    if(fname.size() > 0)
        ocv->openVideo(fname);
}

void MainWindow::loadCamera()
{
    ocv->openCam();
}

void MainWindow::aboutDlg()
{
    QString dlgtext;
    dlgtext  = "Image Processing and Computer Vision Development Toolbox.\n\n";
    dlgtext += "Copyright (C) University of Hawaii at Manoa\n";
    dlgtext += "Human-Robot Interaction Laboratory";

    QMessageBox aboutDlg;
    aboutDlg.setWindowTitle("QtOpenCV devel");
    aboutDlg.setText(dlgtext);
    aboutDlg.setStandardButtons(QMessageBox::Ok);
    aboutDlg.exec();
}

void MainWindow::updatePrincipalPoint()
{
    ogl->AUTO_CAL_X = ocv->ip->getPrincipalPoint().x;
    ogl->AUTO_CAL_Y = ocv->ip->getPrincipalPoint().y;
}
