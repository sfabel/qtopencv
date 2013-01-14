#include "pPOSIT.h"

#include <iostream>
using std::cout;
using std::endl;

#ifdef Q_OS_WIN32
#include <cstdlib>
using std::srand;

#include <ctime>
using std::time;
#endif

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>

//using namespace cv;

pPOSIT::pPOSIT( CvPOSITObject* a, vector<CvPoint2D32f> b , double c, CvTermCriteria d )
        : positObject( a ), srcImagePoints( b ), fl( c ), criteria( d )
{
    rotation_matrix = new float[9];
    translation_vector = new float[3];
}

pPOSIT::~pPOSIT()
{

}

void pPOSIT::runEstimator( vector<CvPoint2D32f> srcImagePoints)
{
    cvPOSIT( positObject , &srcImagePoints[0], fl, criteria, rotation_matrix, translation_vector );


#ifdef DEBUG
    cout << endl << "DEBUG: " << "translation vector now: \n";
    cout << "\t" << translation_vector[0] << " | " << translation_vector[1] << " | " << translation_vector[2] << endl << endl;

    cout << "DEBUG: " << "rotation matrix now \n";
    for (int i=0; i<9; i=i+3)
        cout << "\t" << rotation_matrix[i] << " | " << rotation_matrix[i+1] << " | " << rotation_matrix[i+2] << endl << endl;
#endif
}

CvVect32f pPOSIT::getTvec() const
{
    return translation_vector;
}

CvMatr32f pPOSIT::getRmat() const
{
    return rotation_matrix;
}

std::vector<CvPoint2D32f> pPOSIT::setImagePoints(vector<CvPoint2D32f> srcImagePoints)
{
#ifdef Q_OS_WIN32
    srand ( time(0) );
    ::Sleep(0.1);
#endif
#ifdef Q_OS_LINUX
    srand ( time(NULL) );
    usleep(100000);
#endif
    int mynumberx = 0;
    int mynumbery = 0;

    if((rand() % 10 + 1) > 5) {
        mynumberx += rand() % 10 + 1;
        mynumbery += rand() % 10 + 1;
    }
    else {
        mynumberx -= rand() % 10 + 1;
        mynumbery -= rand() % 10 + 1;
    }
    cout << "x: " << mynumberx << endl;
    cout << "y: " << mynumbery << endl;
    cout << "newImgPts:" << endl;

    for (size_t i=0;i<srcImagePoints.size();++i)
    {
            srcImagePoints[i].x = srcImagePoints[i].x + mynumberx;
            srcImagePoints[i].y = srcImagePoints[i].y + mynumbery;
    }
    return srcImagePoints;
}

void pPOSIT::printResults() const
{
    cout << "translation vector now: \n";
    cout << "\t" << translation_vector[0] << " | " << translation_vector[1] << " | " << translation_vector[2] << endl << endl;

    cout << "rotation matrix now \n";
    for (int i=0; i<9; i=i+3)
        cout << "\t" << rotation_matrix[i] << " | " << rotation_matrix[i+1] << " | " << rotation_matrix[i+2] << endl;
    cout << endl;
}


