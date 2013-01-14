#include <armadillo>
using arma::field;
using arma::vec;
using arma::mat44;

#include <vector>
using std::vector;

#include <opencv/cxcore.h>
#include <opencv/cv.h>
using cv::Point2i;
using cv::Point3f;

class QMatrix4x4;
class PoseLM;

class P3P {

public:
    P3P( vector< Point3f >, vector< Point2i >, vector< double >, double);
    ~P3P();

    void runEstimator();
    void printResults();

    // various get/set methods
    inline field< vec > getImagePointsArma() const  {
        return ImgPts;
    }

    inline vector< Point2i > getImagePoints() const {
        return ImagePoints;
    }

    void setImagePoints(vector< Point2i >);
    void setImagePointsArma(field< vec >);

    inline vector< Point3f > getObjPoints() const {
        return ObjectPoints;
    }

    inline void setObjPoints(vector< Point3f> new_val) {
        if(new_val.size() == ObjectPoints.size() )
            ObjectPoints = new_val;
    }

    field< vec > getPoseArma() const;
    vector< Point3f > getPose() const;

    QMatrix4x4 getRegister(vector< Point3f >, vector< Point3f >);

private:
    // estimator
    PoseLM *plm;

    // arma structures
    field< vec > ImgPts;
    field< vec> objPts;

    // std / opencv structures
    vector< Point3f > ObjectPoints;
    vector< Point2i > ImagePoints;
};
