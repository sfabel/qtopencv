#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <iostream>
#include <vector>

using namespace cv;

#undef DEBUG

class pPOSIT
{
private:
	CvPOSITObject* positObject;
	std::vector<CvPoint2D32f> srcImagePoints;
	double fl;
	CvTermCriteria criteria;
	CvMatr32f rotation_matrix;
        CvVect32f translation_vector;

public:
        pPOSIT( CvPOSITObject*, std::vector<CvPoint2D32f>, double, CvTermCriteria );
	
	~pPOSIT();

        void runEstimator( std::vector<CvPoint2D32f> );
        void printResults() const;
        // get/set methods
        CvVect32f getTvec() const;
        CvMatr32f getRmat() const;
        vector<CvPoint2D32f> setImagePoints( std::vector<CvPoint2D32f>);

};
