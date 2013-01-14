#include "qtopencv.h"

/** qtopencv()
 * \brief qtopencv constructor.
 *
 * This is a QWidget derived class representing the interface to the
 * OpenCV world for a Qt program. The goal is to separate OpenCV
 * functionality from the Qt portion of the program and this class
 * glues both worlds together.
 *
 * @param parent *QWidget pointer to parent of the qtopencv widget
 * @see resize()
 */
qtopencv::qtopencv(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
    setWindowTitle(tr("CLSR Instrument Registration"));
    ui.pushButton_Quit->setDefault(true);

    ip = new Image_Processing();
    //ip->captureFrame(false, false, false);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(ocv2qt()));

    normal_mode = true;
    autocalib_mode = false;
    pose_mode = false;
    extract_mode = false;

    pixmap = new QPixmap();

    //ocv_image.create( ip->getSize(), ip->getType() );

    width  = ip->getSize().width;
    height = ip->getSize().height;

    ui.spinBox_height->setValue( height );
    ui.spinBox_height->setMaximum( height );
    ui.spinBox_width->setValue( width );
    ui.spinBox_width->setMaximum( width );

    connect(ui.pushButton_startCapt, SIGNAL(clicked()), this, SLOT(captureStarted()));
    connect(ui.pushButton_stopCapt, SIGNAL(clicked()), this, SLOT(captureStopped()));
    connect(ui.spinBox_height, SIGNAL(valueChanged(int)), this, SLOT(setHeight(int)));
    connect(ui.spinBox_width, SIGNAL(valueChanged(int)), this, SLOT(setWidth(int)));
    connect(ui.pushButton_Calibration,SIGNAL(clicked()),this,SLOT(doCalib()));
    connect(ui.checkBox_undistort,SIGNAL(toggled(bool)),this,SLOT(doUndistort(bool)));

    connect(ui.checkBox_Jon,SIGNAL(toggled(bool)),this,SLOT(activatePose(bool)));
    connect(ui.checkBox_tony,SIGNAL(toggled(bool)),this,SLOT(activateExtract(bool)));
    connect(ui.checkBox_yang,SIGNAL(toggled(bool)),this,SLOT(activateAutoCalib(bool)));
    connect(ui.slider_Threshold, SIGNAL(valueChanged(int)), this, SLOT(setThreshold(int)));
}

/** ~qtopencv()
 * \brief destructor; just deletes the ImageProcessing object before
 * cleaning up.
 *
 */
qtopencv::~qtopencv()
{
    delete ip;
}


/** ocv2qt()
 * \brief reads current frame and converts it to a QImage.
 *
 * This method checks which mode is enabled and sets the mode variable
 * correspondingly. The current frame is "copied" using cvCopy instead
 * of cvClone which just sets the data_ptr correctly and is thus
 * faster.
 *
 * The QImage is then created with the data_ptr as parameter so no
 * actual data is copied. This conversion is thus really fast and able
 * to convert frames at real-time from OpenCV IplImage to QImage.
 *
 */
void qtopencv::ocv2qt()
{
    int error = ip->captureFrame(autocalib_mode,pose_mode,extract_mode);

    if(error != 0)
        return;

    ip->getFrame().copyTo( ocv_image );

    // point to the image data stored in the IplImage*
    const unsigned char * data = ocv_image.data;

    // read other parameters in local variables
    int ocvwidth 	= ocv_image.size().width;
    int ocvheight	= ocv_image.size().height;
    int bytesPerLine 	= ocv_image.step;

    qimage = QImage( data, ocvwidth, ocvheight, bytesPerLine, QImage::Format_RGB32 );

    if( ocvwidth != width || ocvheight != height )
        qimage = qimage.scaled( width, height, Qt::KeepAspectRatio );

    ui.imageframe->setPixmap(pixmap->fromImage(qimage, 0));

    if(ip->isAutoCalibOn())
        emit autoCalib();

    if(ip->isFeatureExtractionOn())
        emit featureExtraction();

    if(ip->isPoseEstimationOn())
        emit poseEstimation();

    return;
}

void qtopencv::captureStarted()
{
//    timer->start(1000/ip->getFPS());
    timer->start(10);
}


void qtopencv::captureStopped()
{
    timer->stop();
}

void qtopencv::doCalib()
{
    ip->doCalibration();
}

void qtopencv::doAutoCalib()
{
    ip->doAutoCalibration();
}

void qtopencv::doUndistort(bool flag)
{
        ip->doUndistort(flag);
}

void qtopencv::activateAutoCalib(bool t)
{
    autocalib_mode = t;
}

void qtopencv::activateExtract(bool t)
{
    extract_mode = t;
}

void qtopencv::activateNormal(bool t)
{
    normal_mode = t;
    ui.checkBox_Jon->setChecked(false);
    ui.checkBox_tony->setChecked(false);
    ui.checkBox_yang->setChecked(false);
}

void qtopencv::activatePose(bool t)
{
    pose_mode = t;
}

void qtopencv::openCam()
{
    this->captureStopped();
    ip->openCAM();
    this->captureStarted();
}

void qtopencv::openImage(QString fname)
{
    this->captureStopped();
    ip->openImageFile(fname);
    this->captureStarted();
}

void qtopencv::openVideo(QString fname)
{
    this->captureStopped();
    ip->openVideoFile(fname);
    this->captureStarted();
}

void qtopencv::setThreshold(int t)
{
    ip->Calib_Threshold = t;
}
