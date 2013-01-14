#include "Reg3D.h"

#undef AUTOMATIC_VALUES

Reg3D::Reg3D(field< vec > ptsA, field< vec > ptsB)
        : PointsA( ptsA ), PointsB( ptsB )
{
    if (PointsA.n_elem != PointsB.n_elem)
    {
        cerr << "ERR( Reg3D ): # of points in either set must match!" << endl;
        exit(EXIT_FAILURE);
    }
    else
    {
        nPts = PointsA.n_elem;
    }

    rotMatrix.set_size(3,3);
    traVector.set_size(3);    

    // generate gaussian noise
    N.set_size(nPts);
    for (size_t i=0;i<nPts;++i)
        N(i) = randn<mat>(3,3);

    doRegistration();
}


Reg3D::~Reg3D() { }

void Reg3D::doRegistration()
{
    wall_clock timer;
    timer.tic();

    vec centroidA = zeros(3);
    vec centroidB = zeros(3);

#ifdef AUTOMATIC_VALUES
    PointsA.print("PointsA: ");
    PointsB.print("PointsB: ");
#endif

    // 1/N sum( p'), 1/N sum( p )
    for (size_t i=0;i<nPts;++i)
        centroidA += PointsA(i);
    centroidA /= static_cast<double>(nPts);

    for (size_t i=0;i<nPts;++i)
        centroidB += PointsB(i);
    centroidB /= static_cast<double>(nPts);

#ifdef AUTOMATIC_VALUES
    centroidA.print("p");
    centroidB.print("p'");
#endif

    field< vec > qA(nPts);
    field< vec > qB(nPts);
    for (size_t i=0;i<nPts;++i)
    {
        qA(i).set_size(3);
        qB(i).set_size(3);
        qA(i) = PointsA(i) - centroidA;
        qB(i) = PointsB(i) - centroidB;
    }

#ifdef AUTOMATIC_VALUES
    qA.print("qA");
    qB.print("qB'");
#endif

    mat H(3,3);
    H = zeros(3,3);
    for (size_t i=0;i<nPts;++i)
        H += qA(i) * trans(qB(i));
    H /= 2.0; // I don't know why but this has to be there ...

    H.save("covarianceH.mat", raw_ascii);

#ifdef AUTOMATIC_VALUES
    H.print("H:");
#endif

    mat U(3,3);
    mat Vt(3,3);
    mat V(3,3);
    colvec s(nPts);

    if(!svd(U, s, Vt, H))
        cerr << "SVD returned false!" << endl;

    V = trans(Vt);           // H = U*A*trans(V)

#ifdef AUTOMATIC_VALUES
    V.print("V:");
    U.print("U:");
    s.print("s:");
#endif

    mat X(3,3);
    int k = 0;
    unsigned int zeropos;

    X = V * trans(U);
    //X.print("X");

    if (round(det(X)) == -1.0)
    {

        // count zeros
        for (size_t i=0;i<nPts;++i)
            if ( fabs(s(i)) < 10e-10 )//25*math::eps() )
                k++, zeropos = i;

        switch ( k )
        {
        case 0:
            // abort with error - the qA(i) are collinear
            cerr << "ERR( Reg3D ): no registration using SVD possible!" << endl;
            s.print("s:");
            exit(EXIT_FAILURE);
            break;
        case 1:
            // coplanar case
            V.col(zeropos) *= -1.0;
#ifdef AUTOMATIC_VALUES
            V.print("Adjusted V:");
#endif
            // fall through
        default:
            X = V * trans(U);
            break;
        }
    }
    else if(round(det(X)) != 1.0)
    {
        cerr << "ERR( Reg3D ): no registration possible, determinant not +/- 1: " << det(X)  << endl;
        exit(EXIT_FAILURE);
    }

    rotMatrix = X;

    traVector = centroidB - rotMatrix*centroidA;
    double elapsed_time = timer.toc();
    cout << "REG3D TIME (sec): " << elapsed_time << endl;
    homTransform.submat(0,0,2,2) = rotMatrix;
    homTransform.submat(0,3,2,3) = traVector;
    homTransform.submat(3,0,3,2) = zeros<rowvec>(3);
    homTransform(3,3) = 1.0;

    double reg_error = 0.0;
    for(size_t i=0;i<3;++i)
        reg_error += norm(PointsA(i) - (rotMatrix*PointsB(i) + traVector),2);

    cerr << "MSG( Reg3D ): Final Registration Error: " << reg_error << endl;

    calcParameters();
}

void Reg3D::calcParameters()
{

    double r11 = rotMatrix(0,0);
    double r12 = rotMatrix(0,1);
    //double r13 = rotMatrix(0,2);
    double r21 = rotMatrix(1,0);
    double r22 = rotMatrix(1,1);
    //double r23 = rotMatrix(1,2);
    double r31 = rotMatrix(2,0);
    double r32 = rotMatrix(2,1);
    double r33 = rotMatrix(2,2);

    beta  = atan2( -r31, sqrt(pow(r11,2)+pow(r21,2)) );
    if ( fabs(cos(beta)) > arma::math::eps() )
    {
        gamma = atan2( r21/cos(beta), r11/cos(beta) );
        alpha = atan2( r32/cos(beta), r33/cos(beta) );
    }
    else     // degenerate case, beta = +/- 90 deg
    {
        cerr << "WARN( Reg3D ): degenerate case, assuming gamma = 0" << endl;
        gamma = 0;
        cos(beta-arma::math::pi()/4) > 0 ? alpha = atan2( r12, r22 ) : alpha = -atan2( r12, r22 );
    }

    tx = traVector(0);
    ty = traVector(1);
    tz = traVector(2);

#ifdef AUTOMATIC_VALUES
    double angle = acos( (r11+r22+r33-1.0)/2.0 );
    vec K(3);
    K(0) = r32-r23;
    K(1) = r13-r31;
    K(2) = r21-r12;
    K = K * 1.0/(2*sin(angle));

    cout << "EULER Angles    : alpha: " << alpha << ", beta: " << beta << ", gamma: " << gamma << endl;
    cout << "EULER Parameters: theta: " << angle << endl;
    K.print("K");
#endif
}

mat Reg3D::getRotation()
{
    return rotMatrix;
}

vec Reg3D::getTranslation()
{
    return traVector;
}

mat44 Reg3D::getTransformation()
{
    return homTransform;
}

vector< double > Reg3D::getParameters()
{
    vector< double > retVect;
    retVect.push_back( alpha );
    retVect.push_back( beta );
    retVect.push_back( gamma );
    retVect.push_back( tx );
    retVect.push_back( ty );
    retVect.push_back( tz );

    return retVect;
}
