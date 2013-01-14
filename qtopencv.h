#ifndef QTOPENCV_H
#define QTOPENCV_H

#include <QtGui/QWidget>
#include <QtCore/QTimer>
#include "ui_qtopencv.h"

#include "Image_Processing.h"

class qtopencv : public QWidget
{
    Q_OBJECT

    friend class MainWindow;

public:
    qtopencv(QWidget *parent = 0);
    ~qtopencv();

    inline QImage* getImage() const {
        return const_cast<QImage *>(&qimage);
    }

    void openVideo(QString);
    void openCam();
    void openImage(QString);

public slots:
    inline void setWidth( int w ) {
        if(w>0)
            width = w;
    }

    inline void setHeight( int h ) {
        if(h>0)
            height = h;
    }

    inline void toggleDrawCalib(bool t) {
        ip->toggleDrawCalib(t);
    }

    inline void toggleDrawFeatures(bool t) {
        ip->toggleDrawFeatures(t);
    }

    inline void toggleDrawPose(bool t) {
        ip->toggleDrawPose(t);
    }

signals:
    void autoCalib();
    void featureExtraction();
    void poseEstimation();

private:
    Ui::qtopencvClass 	 ui;
    Image_Processing	*ip;
    QTimer 		        *timer;
    QPixmap             *pixmap;
    Mat                  ocv_image;
    QImage	             qimage;
    bool		         normal_mode;
    bool                 autocalib_mode;
    bool                 pose_mode;
    bool                 extract_mode;
    int                  width;
    int                  height;

private slots:
    void captureStarted();
    void captureStopped();
    void ocv2qt();
    void doCalib();
    void doAutoCalib();
    void doUndistort(bool);
    void activateAutoCalib(bool);
    void activateNormal(bool);
    void activatePose(bool);
    void activateExtract(bool);
    void setThreshold(int t);
  };

#endif // QTOPENCV_H
