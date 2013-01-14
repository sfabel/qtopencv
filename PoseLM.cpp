#include "PoseLM.h"

PoseLM::PoseLM( field< vec > imgPts,
                field< vec> objPts,
                vec pxPitch,
                double focal_length,
                vec initial_values,
                vec err_lim )
 :  PoseEstimator( imgPts, objPts, pxPitch, focal_length, initial_values ),
    eps( err_lim )
{
    H.set_size(3,3);
    dp.set_size(3);

    k = 0;
    nu = 2;
    p = p0;

    J = calcJac(p0);
    H = trans(J)*J;
    e = f(p0);
    g = trans(J) * e;

    stop = max(max(abs(g))) < eps(0);
    myu = 0.01 * max(max(J));

    runEstimator();
}

PoseLM::~PoseLM() { }

void PoseLM::runEstimator()
{
#ifdef Q_OS_LINUX
    wall_clock timer;
    timer.tic();
#endif
    while ( ++k < ITER_MAX && !stop ) {
        do {

            dp = inv( H + myu*eye(3,3) ) * g;

            if (norm(dp,2) < eps(1)*norm(p,2)) {
                //cout << "\nnorm(dp,2) < eps(1)*norm(p,2): " << norm(dp,2) << " < " << eps(1)*norm(p,2) << endl;
                stop = true;
            } else {
                p_n = p - dp;
                rho = (pow(norm(e,2),2)-pow(norm(f(p_n),2),2))/(dot(trans(dp),myu*dp+g));

                if ( norm(f(p_n),2) < norm(f(p),2) ) {
                    p = p_n;
                    J = calcJac(p);
                    H = trans(J) * J;
                    e = f(p);
                    g = trans(J) * e;
                    myu *= std::max(0.33, 1-pow(2*rho-1,3));
                    nu = 2;
                } else {
                    myu *= nu;
                    nu *= 2;
                }
            }
        } while ( rho < 0 && !stop );
    }
#ifdef Q_OS_LINUX
    double elapsed_time = timer.toc();
    cout << "POSE_LM TIME (sec): " << elapsed_time << endl;
#endif
    estPts(0) = p(0) * q(0);
    estPts(1) = p(1) * q(1);
    estPts(2) = p(2) * q(2);

/*    if((estPts(0)(2) < 0) && (estPts(1)(2) < 0) && (estPts(2)(2) < 0))
        for(size_t i=0;i<3;++i)
            estPts(i) = estPts(i) * (-1.0);
    else if ((estPts(0)(2) > 0) && (estPts(1)(2) > 0) && (estPts(2)(2) > 0))
        true;
    else
        cerr << "ERR( PoseLM ): Some points are estimated before, some behind the origin!" << endl;
*/
}

void PoseLM::printResults() const
{
    //estPts.print("P3P with LM Result:");

    vector< double > edist;
    edist.push_back(norm(estPts(0)-estPts(1),2));
    edist.push_back(norm(estPts(1)-estPts(2),2));
    edist.push_back(norm(estPts(0)-estPts(2),2));

    //cout << "MSG( PoseLM ): Given object distances:     " << x.at(0) << " " << x.at(1) << " " << x.at(2) << endl;
    cout << "MSG( PoseLM ): Estimated object distances: " << edist.at(0) << " " << edist.at(1) << " " << edist.at(2) << endl;
    cerr << "MSG( PoseLM ): # iter: " << k << "\n" << endl;
}
