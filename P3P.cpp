#include <QMatrix4x4>

#include "P3P.h"
#include "PoseLM.h"
#include "Reg3D.h"


P3P::P3P( vector< Point3f > ObjPoints, vector< Point2i > ImgPoints, vector< double > initialGuess, double fl)
    : ObjectPoints(ObjPoints), ImagePoints(ImgPoints)
{
    // set number of image points for arma structure
    ImgPts.set_size(3);
    field< vec> objPts(3);

    vec pxPitch = ".0075 .00825 .0075";
    vec initial;
    initial.set_size(3);

    for (size_t i=0;i<3;++i)
    {
        ImgPts(i).set_size(2);
        ImgPts(i)(0) = ImagePoints[i].x;
        ImgPts(i)(1) = ImagePoints[i].y;
    }

    for (size_t i=0;i<3;++i)
    {
        objPts(i).set_size(3);
        objPts(i)(0) = ObjectPoints[i].x;
        objPts(i)(1) = ObjectPoints[i].y;
        objPts(i)(2) = ObjectPoints[i].z;
    }

    initial(0) = initialGuess.at(0);
    initial(1) = initialGuess.at(1);
    initial(2) = initialGuess.at(2);

    vec err_lim = "1e-55 1e-55 1e-55";

    plm = new PoseLM( ImgPts, objPts, pxPitch, fl, initial, err_lim );
}

P3P::~P3P()
{
    delete plm;
}

void P3P::runEstimator()
{
    plm->runEstimator();
}

void P3P::printResults()
{
    plm->printResults();
}

void P3P::setImagePoints(vector< Point2i > new_val) {

    if(new_val.size() == ImagePoints.size())
        ImagePoints = new_val;

    cout << "newImgPts:" << endl;
    // sync both structures so they contain same point data
    for(unsigned int i=0;i<ImagePoints.size();++i) {
        ImgPts(i)(0) = new_val.at(i).x;
        ImgPts(i)(1) = new_val.at(i).y;

        cout << "\t\t" << new_val[i].x << "," << new_val[i].y << endl;
    }

    //ImgPts.print("DEBUG IMAGE POINTS");
    // set the points in the estimator
    plm->setImgPts(ImgPts);
}

void P3P::setImagePointsArma(field< vec > new_val) {

    if(new_val.n_elem == ImgPts.n_elem)
        ImgPts = new_val;

    // sync both structures so they contain same point data
    for(unsigned int i=0;i<ImgPts.n_elem;++i) {
        ImagePoints[i].x = ImgPts(i)(0);
        ImagePoints[i].y = ImgPts(i)(1);
    }

    // set the points in the estimator
    plm->setImgPts(ImgPts);
}

field< vec > P3P::getPoseArma() const
{    
    return plm->getPose();
}

vector< Point3f > P3P::getPose() const
{
    vector< Point3f > posePoints(3);
    field< vec > pPts(3);
    pPts = plm->getPose();
    for(unsigned int i=0;i<pPts.n_elem;++i) {
        posePoints[i].x = pPts(i)(0);
        posePoints[i].y = pPts(i)(1);
        posePoints[i].z = pPts(i)(2);
    }
    return posePoints;
}


QMatrix4x4 P3P::getRegister(vector< Point3f > ObjPoints, vector< Point3f > estimatedPoints)
{
    field< vec > objPts(3);
    field< vec > estPts(3);

    for (size_t i=0;i<3;++i)
    {
        objPts(i).set_size(3);
        objPts(i)(0) = ObjPoints[i].x;
        objPts(i)(1) = ObjPoints[i].y;
        objPts(i)(2) = ObjPoints[i].z;
    }

    for (size_t i=0;i<3;++i)
    {
        estPts(i).set_size(3);
        estPts(i)(0) = estimatedPoints[i].x;
        estPts(i)(1) = estimatedPoints[i].y;
        estPts(i)(2) = estimatedPoints[i].z;
    }

    estPts.print("P3P with LM Result:");

    Reg3D A(objPts,estPts);
    mat44 homTransform = A.getTransformation();

    QMatrix4x4 transMat(homTransform.at(0,0),
                         homTransform.at(0,1),
                         homTransform.at(0,2),
                         homTransform.at(0,3),
                         homTransform.at(1,0),
                         homTransform.at(1,1),
                         homTransform.at(1,2),
                         homTransform.at(1,3),
                         homTransform.at(2,0),
                         homTransform.at(2,1),
                         homTransform.at(2,2),
                         homTransform.at(2,3),
                         homTransform.at(3,0),
                         homTransform.at(3,1),
                         homTransform.at(3,2),
                         homTransform.at(3,3));

    return transMat;
}














/*
field< vec > P3P::setImagePoints(vector< Point2i > ImagePoints)
{
    field< vec > newImgPts(ImagePoints.size());
#ifdef Q_OS_WIN32
    srand ( time(0) );
    ::Sleep(0.1);
#endif
#ifdef Q_OS_LINUX
    srand ( time(NULL) );
    usleep(100000);
#endif

    int mynumberx = rand() % 10 + 1;
    int mynumbery = rand() % 10 + 1;

    for (size_t i=0;i<3;++i)
    {
        newImgPts(i).set_size(2);
        newImgPts(i)(0) = ImagePoints[i].x + mynumberx;
        newImgPts(i)(1) = ImagePoints[i].y + mynumbery;
    }

    plm->setImgPts( newImgPts );
    return newImgPts;
}

vector< Point2i > P3P::getImgPoints(vector< Point2i > ImagePoints)
{
        srand ( time(NULL) );
        usleep(100000);
        int mynumberx = rand() % 10 + 1;
        int mynumbery = rand() % 10 + 1;

        for (int i=0;i<3;++i)
        {
                ImagePoints[i].x += mynumberx;
                ImagePoints[i].y += mynumbery;
        }
        return ImagePoints;
}

#ifdef DEBUG
        cout << endl << "DEBUG: " << "translation vector now: \n";
        cout << "\t" << translation_vector[0] << " | " << translation_vector[1] << " | " << translation_vector[2] << endl << endl;

        cout << "DEBUG: " << "rotation matrix now \n";
        for (int i=0; i<9; i=i+3)
                cout << "\t" << rotation_matrix[i] << " | " << rotation_matrix[i+1] << " | " << rotation_matrix[i+2] << endl << endl;
#endif
}

void P3P::printResults() const
{
        cout << "translation vector now: \n";
        cout << "\t" << translation_vector[0] << " | " << translation_vector[1] << " | " << translation_vector[2] << endl << endl;

        cout << "rotation matrix now \n";
        for (int i=0; i<9; i=i+3)
                cout << "\t" << rotation_matrix[i] << " | " << rotation_matrix[i+1] << " | " << rotation_matrix[i+2] << endl;
        cout << endl;
}



        for(int i=0;i<9;++i) {
                cout << "\t" << " " << rotation_matrix[i] << " ";
                if(i==2 || i==5)
                        cout << "\n";
        }
        cout << endl;

*/

