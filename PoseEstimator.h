#ifndef POSEESTIMATOR_H
#define POSEESTIMATOR_H

#include <vector>
using std::vector;

#include <armadillo>
using arma::field;
using arma::vec;
using arma::mat;
using arma::norm;
using arma::zeros;

/// class PoseEstimator
/// \brief Abstract class providing P3P pose estimation.
///
/// This class provides the basic means of P3P estimation. It is used by the concrete
/// implementation classes PoseLM and PoseRN, specifying the kind of minimization
/// algorithm that is chosen by the user.
///
/// @see PoseLM
/// @see PoseRN
class PoseEstimator
{
    public:
        PoseEstimator(  field< vec > imgPts,
                        field< vec > objPts,
                        vec pxPitch,
                        double focal_length,
                        vec initial_values );

        virtual ~PoseEstimator();

        // set methods
        void setImgPts( field< vec > newImgPts );

        // get methods
        field< vec > getPose() const;
        vec getParams() const;

    protected:
        field< vec > iPts;
        const field< vec > oPts;
        const vec px2si;
        const double fl;
        const vec p0;

        field< vec > estPts;
        field< vec > q;
        field< vec > Q;

        vector< double > x;

        mat J;
        vec e;
        vec p;
        vec p_n;

        mat calcJac( vec A );
        vec f( vec A );
        void initAttributes();

        virtual void runEstimator();
        virtual void printResults() const = 0;
};

#endif // POSEESTIMATOR_H
