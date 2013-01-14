#ifndef REG3D_H
#define REG3D_H

#include <QMatrix4x4>

#include <vector>
using std::vector;

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <cmath>
using std::fabs;

#include <cstdlib>

#include <armadillo>
using arma::mat;
using arma::mat44;
using arma::vec;
using arma::field;
using arma::randn;
using arma::wall_clock;
using arma::zeros;
using arma::raw_ascii;
using arma::colvec;
using arma::rowvec;

class Reg3D
{
    public:
        Reg3D(field< vec > ptsA, field< vec > ptsB);

        virtual ~Reg3D();

        mat getRotation();
        vec getTranslation();

        mat44 getTransformation();

        vector< double > getParameters();

    private:
        const field< vec > PointsA;
        const field< vec > PointsB;

        mat rotMatrix;
        vec traVector;
        mat44 homTransform;

        double alpha;
        double beta;
        double gamma;
        double tx;
        double ty;
        double tz;

        size_t nPts;

        field< mat > N;

        void doRegistration();
        void calcParameters();
};

#endif // REG3D_H
