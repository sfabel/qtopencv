#ifndef IMAGE_PROCESSING_H_
#define IMAGE_PROCESSING_H_

#include <QFile>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QXmlStreamReader>
#include <QtXml/QDomDocument>
#include <QtXmlPatterns/QXmlNamePool>
#include <QMatrix4x4>
#include <QDebug>
#include <QMessageBox>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
using namespace cv;

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <vector>
using std::vector;

#include <cmath>
using std::fabs;

#include <ctime>
using std::time;

#include <algorithm>
using std::max;


class P3P;
class pPOSIT;
#undef estP3P
#define estPOSIT

#include "FindROI.h"

// modes of operation
const unsigned int NORMAL_OPERATION   = 0;
const unsigned int YANG               = 1;
const unsigned int TONY               = 2;
const unsigned int JON                =	3;


class Image_Processing
{
public:
    int Calib_Threshold;
    Image_Processing();
    virtual ~Image_Processing();
    int captureFrame(bool autocalib_mode = false, bool pose_mode = false, bool extract_mode = false);
    void doCalibration();
    void doAutoCalibration();
    void doUndistort(bool flag);
    int calimain();

    inline Point2f getPrincipalPoint() const {
        return centerpoint;
    }

    inline Mat getFrame() {
        return dframe;
    }

    inline Size2i getSize() {
        return frame_size;
    }

    inline int getType() {
        return dframe.type();
    }

    inline double getFPS() {
        return fabs(fps);
    }

    inline void toggleDrawCalib(bool t) {
        draw_autocalib = t;
    }

    inline void toggleDrawFeatures(bool t) {
        draw_features = t;
    }

    inline void toggleDrawPose(bool t) {
        draw_pose = t;
    }

    inline QMatrix4x4 getTransMat() const {
        return transMat;
    }

    inline bool isAutoCalibOn() const {
        return autocalib_mode;
    }

    inline bool isPoseEstimationOn() const {
        return pose_mode;
    }

    inline bool isFeatureExtractionOn() const {
        return extract_mode;
    }

    void openCAM();
    void openVideoFile(QString);
    void openImageFile(QString);

private:
    bool autocalib_mode;
    bool pose_mode;
    bool extract_mode;
    int find_blob();
    int initAttributes();
    void pEstimate();
    void FindROI();
    void SaveContours();
    void findROI();
    void AddPoints();
    void MatchContours();
    double fl;
    int CUBE_SIZE;

    vector<CvPoint3D32f> modelPoints;
    vector<CvPoint2D32f> srcImagePoints;
    vector< Point3f > ObjectPoints;
    vector< Point2i > ImagePoints;
    vector< double > initialGuess;
    pPOSIT* p;
    P3P* q;
    QMatrix4x4 transMat;

    // image processing stuff tony
    QHash<int, vector< Point > > hashL;
    QHash<int, vector< Point > > hashR;
    int point_Left_x, point_Left_y, offset_Left_x, offset_Left_y;
    int point_Right_x, point_Right_y, offset_Right_x, offset_Right_y;
    Mat uframe_roiL, uframe_roiR;
    Rect roiL,roiR;

    VideoCapture  capture;
    double	  brightness, hue, contrast, saturation;
    Size2i        frame_size;
    bool          cali_in_progress;
    bool          undistort_flag;
    double        fps;
    bool          cam_flag;
    bool          img_flag;
    bool          vdo_flag;
    QString       current_vdo_file;
    QString       current_img_file;

    bool      draw_autocalib;
    bool      draw_features;
    bool      draw_pose;

    Mat       dframe;
    Mat       tframe;
    Mat       cframe;
    Mat       gframe;
    Mat       caldframe; //added by yang; calibrated
    Mat       uframe;    //undistorted frame

    Point2f  centerpoint;

    // the extra channels for the IplImage
    vector< Mat > channels;
    Mat intrinsic_matrix;
    Mat distortion_coeffs;
    double fx, fy, cx, cy, k1, k2, k3, p1, p2;
    double rx, ry, rz, tx, ty, tz;
    //Mat intrinsic_matrix(3,3,CV_32FC1);
    //Mat distortion_coeffs(5,1,CV_32FC1); //add by yang:fx, fy, cx, cy, and K1,K2,K3,P1,P2
   int Mat_To_XML();
   bool XML_To_Mat();
};

#endif /*IMAGE_PROCESSING_H_*/
