#ifndef POSELM_H
#define POSELM_H

#include "PoseEstimator.h"

#include <armadillo>
using arma::trans;
using arma::max;
using arma::wall_clock;
using arma::inv;
using arma::dot;
using arma::eye;

#include <iostream>
using std::cout;
using std::endl;
using std::cerr;

#define ITER_MAX 100

/// class PoseLM
/// \brief Child of PoseEstimator, using the Levenberg-Marquardt optimization technique for P3P.
///
/// This class uses the well-known Levenberg-Marquardt technique to minimize the error of the
/// parameter vector. Using the "initial_values" arma::vec as a starting point, it then estimates
/// the parameters to find the 3D world points with respect to the camera coordinate system (CCS).
/// For a simpler minimization technique (faster, but maybe more inaccurate), see the PoseRN, which
/// uses Newton-Rhapson minimization.
///
/// @see PoseEstimator
/// @see PoseRN
class PoseLM : public PoseEstimator
{
    public:
        PoseLM( field< vec > imgPts,
                field< vec> objPts,
                vec pxPitch,
                double focal_length,
                vec initial_values,
                vec err_lim );

        virtual ~PoseLM();

        virtual void runEstimator();
        virtual void printResults() const;

    private:
        const vec eps;
        int k;

        double myu;
        double nu;
        double rho;
        bool stop;

        mat H;
        vec g;
        vec dp;
};

#endif // POSELM_H
