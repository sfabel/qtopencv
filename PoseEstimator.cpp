#include "PoseEstimator.h"

PoseEstimator::PoseEstimator(   field< vec > imgPts,
                                field< vec > objPts,
                                vec pxPitch,
                                double focal_length,
                                vec initial_values )
: iPts( imgPts ), oPts( objPts ), px2si( pxPitch ), fl( focal_length ), p0( initial_values )
{
    estPts.set_size(3);
    q.set_size(3);
    Q.set_size(3);
    J.set_size(3,3);
    e.set_size(3);
    p.set_size(3);
    p_n.set_size(3);

    initAttributes();
}

PoseEstimator::~PoseEstimator() { }
void PoseEstimator::runEstimator() { };

void PoseEstimator::initAttributes()
{
    // first, expand the image points with the focal length
    if( iPts(0).n_cols < 3)
        for(size_t i=0;i<iPts.n_elem;++i) {
            double i0 = iPts(i)(0);
            double i1 = iPts(i)(1);
            iPts(i).set_size(3);
            iPts(i)(0) =  i0;
            iPts(i)(1) =  i1;
            iPts(i)(2) = -fl;
        }

    // transform them into points on the lens
    vec i2ftrans = "322 -186 0";

    for(size_t i=0;i<3;++i)
        iPts(i)(1) *= -1;

    // get the image points in [mm] and move the center
    // of the coordinate system to the middle of the
    // lens
    Q(0) = (iPts(0) - i2ftrans) % px2si;
    Q(1) = (iPts(1) - i2ftrans) % px2si;
    Q(2) = (iPts(2) - i2ftrans) % px2si;

    //Q.print("Q: ");

    // calculate unit vectors to the image points
    q(0) = Q(0)/norm(Q(0),2);
    q(1) = Q(1)/norm(Q(1),2);
    q(2) = Q(2)/norm(Q(2),2);

    // create the measured vector x
    x.push_back(norm(oPts(0)-oPts(1),2));
    x.push_back(norm(oPts(1)-oPts(2),2));
    x.push_back(norm(oPts(0)-oPts(2),2));

    // parameter vector
    vec p = p0;
    vec p_n = zeros(3);

    J.set_size(3,3);
    J = calcJac(p0);
    e = f(p0);
}

vec PoseEstimator::f( vec A )
{
    vec xhat(3);

    const double a1 = A(0);
    const double a2 = A(1);
    const double a3 = A(2);

    const double t12 = dot(q(0),q(1));
    const double t23 = dot(q(1),q(2));
    const double t31 = dot(q(2),q(0));

    const double d12 = x.at(0);
    const double d23 = x.at(1);
    const double d13 = x.at(2);

    xhat(0) = a1*a1 - 2*a1*a2*t12 + a2*a2 - d12*d12;
    xhat(1) = a2*a2 - 2*a2*a3*t23 + a3*a3 - d23*d23;
    xhat(2) = a1*a1 - 2*a1*a3*t31 + a3*a3 - d13*d13;

    return xhat;
}

mat PoseEstimator::calcJac( vec A )
{
    mat Jac(3,3);

    const double a1 = A(0);
    const double a2 = A(1);
    const double a3 = A(2);

    const double t12 = dot(q(0),q(1));
    const double t23 = dot(q(1),q(2));
    const double t31 = dot(q(2),q(0));

    Jac(0,0) = 2*a1-2*t12*a2;
    Jac(0,1) = 2*a2-2*t12*a1;
    Jac(0,2) = 0;

    Jac(1,0) = 0;
    Jac(1,1) = 2*a2-2*t23*a3;
    Jac(1,2) = 2*a3-2*t23*a2;

    Jac(2,0) = 2*a1-2*t31*a3;
    Jac(2,1) = 0;
    Jac(2,2) = 2*a3-2*t31*a1;

    return Jac;
}


void PoseEstimator::setImgPts( field< vec > newImgPts )
{
    iPts = newImgPts;
    initAttributes();
}

 field< vec > PoseEstimator::getPose() const
 {
     return estPts;
 }

 vec PoseEstimator::getParams() const
 {
     return p;
 }
