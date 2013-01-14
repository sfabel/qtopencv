#include "Image_Processing.h"
#include "openglscene.h"
#include "P3P.h"
#include "pPOSIT.h"

#include <vector>
using std::vector;

/** Image_Processing()
 * \brief Constructor. Calls initAttributes() and checks OpenCV backend.
 *
 * This is the version using OpenCV 2.0 or higher C++ interface.
 *
 * @see initAttributes()
 *
 */
Image_Processing::Image_Processing()
{
    if(initAttributes() < 0)
        cerr << "\n\n\nError initializing OpenCV backend. Aborting program" << endl;

    Calib_Threshold = 150;

    //Create the POSIT object with the model points
    p = new pPOSIT( cvCreatePOSITObject( &modelPoints[0],
                                         modelPoints.size()),
                    srcImagePoints,
                    fl,
                    cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 1.0e-4f) );

    q = new P3P( ObjectPoints,
                 ImagePoints,
                 initialGuess,
                 fl );

    findROI();
    srand( time(NULL) );
}

/** ~Image_Processing()
 * \brief Destructor.
 *
 */
Image_Processing::~Image_Processing() { }

void Image_Processing::openCAM()
{
    if(vdo_flag) {
        capture.release();
    }

    if(!capture.open(0)) {
        QMessageBox::critical(0,"Open Camera","Could not open camera for capture!");
        qDebug() << "ERROR: Could not open camera for capture!";
        return;
    }
    else {
        capture >> cframe;

        // width and height of frames in the video stream
        frame_size = cframe.size();

        // brightness, contrast, saturation and hue of image
        brightness	= capture.get( CV_CAP_PROP_BRIGHTNESS );
        contrast 	= capture.get( CV_CAP_PROP_CONTRAST );
        saturation	= capture.get( CV_CAP_PROP_SATURATION );
        hue 	    = capture.get( CV_CAP_PROP_HUE );
        fps         = 100;

        qDebug() << "Input Frame Size: " << frame_size.width << "x" << frame_size.height << endl;
    }

    vdo_flag = false;
    img_flag = false;
    cam_flag = true;

    if (XML_To_Mat()) //Added by yang
    {
        //Enable checkBox_undistort
        undistort_flag = true; //actually should enable checkbox and let user to select
    }
}

void Image_Processing::openVideoFile(QString fname)
{
    if(vdo_flag)
        if(current_vdo_file == fname)
            return;

    capture.release();

    if(!capture.open(fname.toStdString())) {
        QMessageBox::critical(0,"Open Camera","Could not open video file!");
        qDebug() << "ERROR: Could not open video file!";
        return;
    }

    qDebug() << "Opening video file " << fname;
    capture >> cframe;
    fps = capture.get(CV_CAP_PROP_FPS);
    qDebug() << "FPS: " << fps;
    if(fps < 1.0)
        fps = 25.0;
    qDebug() << "FPS: " << fps;
    current_vdo_file = fname;
    vdo_flag = true;
    img_flag = false;
    cam_flag = false;
}

void Image_Processing::openImageFile(QString fname)
{
    capture.release();
    current_img_file = fname;
    cframe = imread(fname.toStdString());
    qDebug() << "Opening image file " << fname;
    vdo_flag = false;
    img_flag = true;
    cam_flag = false;
}

/** initAttributes()
 * \brief Initialization of all relevant data structures as needed.
 *
 *
 */
int Image_Processing::initAttributes()
{
    cam_flag = false;
    vdo_flag = false;
    img_flag = false;

    draw_autocalib = false;
    draw_features  = false;
    draw_pose      = false;

    cali_in_progress = 0; //added by yang
    undistort_flag = 0; //added by yang

    frame_size = Size(640,480);

    if(fps < 1.0)
        fps = 100.0;

    fl = 1000.0;
    CUBE_SIZE = 10;

    centerpoint = Point2f(frame_size.width/2,frame_size.height/2);

    //Create the model points
    modelPoints.push_back(cvPoint3D32f(0.0f, 0.0f, 0.0f)); //The first must be (0,0,0)
    modelPoints.push_back(cvPoint3D32f(22.0f, -7.5f, 0.0f));
    modelPoints.push_back(cvPoint3D32f(22.0f,  7.5f, 0.0f));
    modelPoints.push_back(cvPoint3D32f(30.0f,  0.0f, 0.0f));

    //Create the image points
    srcImagePoints.push_back( cvPoint2D32f( 0, 0 ) );
    srcImagePoints.push_back( cvPoint2D32f( -20, 20 ) );
    srcImagePoints.push_back( cvPoint2D32f( -20, -20 ) );
    srcImagePoints.push_back( cvPoint2D32f( 20, 0 ) );

    // gripper object points, not the CUBE, e.g.
    ObjectPoints.push_back(Point3f(0, 0, 0));
    ObjectPoints.push_back(Point3f(22, -7.5, 0));
    ObjectPoints.push_back(Point3f(22,  7.5, 0));

    ImagePoints.push_back(Point2i(320,240));  // first point in the middle
    ImagePoints.push_back(Point2i(300,230));
    ImagePoints.push_back(Point2i(300,250));

    initialGuess.push_back(0);
    initialGuess.push_back(0);
    initialGuess.push_back(300);

    return 0;
}

/** captureFrame( int mode )
 * \brief Captures a single frame and applies optional image filters
 *
 * The important part is to
 * add an alpha channel to the output frame that is accessed by the
 * other parts of the system in order for it to be quickly transformed
 * into a QImage.
 *
 * @param int mode of operation
 * @see qtopencv
 */
int Image_Processing::captureFrame(bool a_mode, bool p_mode, bool e_mode)
{
    autocalib_mode = a_mode;
    pose_mode = p_mode;
    extract_mode = e_mode;

    if(!(cam_flag || vdo_flag || img_flag))
        return -1;

    if( cam_flag ) {
        capture >> cframe; //cframe is a Mat type
    }

    if( vdo_flag ) {
        if( capture.grab() ) {
            capture.retrieve(cframe);
        }
        else {
            capture.set( CV_CAP_PROP_POS_AVI_RATIO, 0 ); // repeat video if at end
            capture >> cframe;
        }
    }

    if( img_flag ) {
        cframe = imread(current_img_file.toStdString());
    }

    // do deinterlacing
    // you need to convert the Mat cframe to an IplImage*
    IplImage fullframe = cframe;
    IplImage * field1 = cvCreateImage( cvSize(cframe.cols,cframe.rows/2),IPL_DEPTH_8U,3);
    IplImage * field2 = cvCreateImage( cvSize(cframe.cols,cframe.rows/2),IPL_DEPTH_8U,3);

    cvDeInterlace(&fullframe, field1, field2);
    Mat temp = cvarrToMat(field2);
    resize(temp,cframe, cframe.size(), 0, 0, INTER_LINEAR);

    cvReleaseImage(&field1);
    cvReleaseImage(&field2);

    // necessary for conversion to QImage later. We need an alpha
    // channel.
    split(cframe, channels);
    channels.push_back(Mat(cframe.size(), CV_8UC1, Scalar(255)));
    merge(channels, cframe);

    if (cali_in_progress){
        uframe = cframe;
        dframe = caldframe;
    }
    else if(undistort_flag) {
        undistort(cframe, uframe,intrinsic_matrix, distortion_coeffs);
        dframe = uframe;
    }
    else {
        uframe = cframe;
        dframe = cframe;
    }

    if(autocalib_mode)
        doAutoCalibration();

    if(extract_mode)
        //FindROI();
        SaveContours();

    if(pose_mode)
        pEstimate();

    return 0;
}


void Image_Processing::doCalibration()
{
    cout << "Image_Processing::doCalibration() is running.." << endl;
    //    cvNamedWindow( "Image View", 1 );
    //    //        IplImage *view = 0, *view_gray = 0;
    //    IplImage fullframe = cframe;
    //    IplImage *view = &fullframe;
    //    cvShowImage( "Image View", view );
    //    //cvReleaseImage( &view );
    //    capture.release();
    //char * argv[5] = {"", "-w", "4", "-h", "4"};

    //persudo_main(5, argv, &cframe, &caldframe, &camera_matrix, &distortion_coeffs);

    cali_in_progress = 1;
    calimain();
    cali_in_progress = 0;

}

int Image_Processing::find_blob()
{
    cout << "find_blob() is running..." << endl;



    /*    IplImage* out = cvCreateImage( cframe.size(), IPL_DEPTH_8U, 1 );

    IplImage src = cframe;
    IplImage* r = cvCreateImage( cvGetSize(&src), IPL_DEPTH_8U, 1 );
    IplImage* g = cvCreateImage( cvGetSize(&src), IPL_DEPTH_8U, 1 );
    IplImage* b = cvCreateImage( cvGetSize(&src), IPL_DEPTH_8U, 1 );
    IplImage* a = cvCreateImage( cvGetSize(&src), IPL_DEPTH_8U, 1 );
    IplImage* t = cvCreateImage( cvGetSize(&src), IPL_DEPTH_8U, 1 );
    // Split image onto the color planes
    cvSplit( &src, r, g, b, a );

    Mat hframe;

    // Perform histogram equalization
    equalizeHist( r, hframe );

    //cvThreshold(g, t, 100, 100, CV_THRESH_TRUNC);
    cvThreshold(r, t, 50, 200, CV_THRESH_BINARY);

    cvSmooth(t,a,CV_MEDIAN,5);*/

    Mat hframe, tframe;
    vector< Mat > chans(cframe.channels());
    split(cframe, chans);

    //Adjust ROI -- Make a rectangle
    Rect roi(120, 0, 480, 300);
    //Point a cv::Mat header at it (no allocation is done)
    Mat image_roi = chans.at(0)(roi);
    equalizeHist( image_roi, hframe );

    threshold(chans.at(0),tframe, Calib_Threshold, 254, CV_THRESH_BINARY);
    cout << "Threshold" <<  Calib_Threshold << endl;

    medianBlur( tframe, tframe, 5);
    //namedWindow("Threshold"); //<<==============================
    //imshow("Threshold", tframe); //<<==============================

    //namedWindow("Histogram"); //<<==============================
    //imshow("Histogram", hframe); //<<==============================

    Mat CTframe = tframe;

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(tframe, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    //Mat outframe = Mat::zeros(cframe.rows, cframe.cols, CV_8UC3);
    RotatedRect rect;
    //Find the largest blob;
    unsigned int s = 6;
    int j = -1;
    for( unsigned int i = 0; i < contours.size(); ++i ) {
        if(contours.at(i).size() > s) {
            j = i;
            s = contours.at(i).size();
        }
    }


    if (j>=0)
    {
        if(contours.at(j).size() > 6) {
            //for debug only


            //--------------------------------
            cout << "Contours found: " << contours.size() << endl;

            Mat c = Mat(contours.at(j));
            rect = fitEllipse(c);
            centerpoint = rect.center;
            if(draw_autocalib) {
                ellipse(dframe, rect, CV_RGB(0,255,0),1,8);
                drawContours(dframe, contours, -1, CV_RGB(0,0,255), 2, CV_AA, hierarchy);//<<=========================
                circle(dframe, centerpoint, 4, CV_RGB(0,255,0), 2, 8);

                //draw center cross lines
                Point pt1=centerpoint, pt2=centerpoint;
                pt1.x -= 20; pt2.x +=20; line(dframe, pt1, pt2, CV_RGB(0,255,0), 1);
                pt1=pt2=centerpoint;
                pt1.y-=20; pt2.y+=20; line(dframe, pt1, pt2, CV_RGB(0,255,0), 1);
            }
            cout << "New Principal Point: (" << centerpoint.x << "," << centerpoint.y << ")" << endl;
        }
        else
            cout << "No contours found!" << endl;
    }//if (j>=0)

    //namedWindow("Contour"); //<<==============================
    //drawContours( dframe, contours, -1, CV_RGB(255,0,0), 2, CV_AA, hierarchy );
    //drawContours(CTframe, contours, -1, CV_RGB(255,255,255), 1, CV_AA, hierarchy);
    //imshow("Contour", CTframe); //<<==============================

    if(draw_autocalib) {
        Point ref1, ref2;
        ref1.x=0; ref1.y=480/2; ref2.x=640; ref2.y=480/2; line(dframe, ref1, ref2, CV_RGB(255,255,0), 1, 8);
        ref1.x=640/2; ref1.y=0; ref2.x=640/2; ref2.y=480; line(dframe, ref1, ref2, CV_RGB(255,255,0), 1, 8);

        if (undistort_flag)
        {
            cx = intrinsic_matrix.at<double>(0,2);
            cy = intrinsic_matrix.at<double>(1,2);
            ref1.x=cx-10; ref1.y=cy;  ref2.x=cx+10; ref2.y=cy; line(dframe, ref1, ref2, CV_RGB(255,255,0), 1, 8);
            ref1.x=cx; ref1.y=cy-10;  ref2.x=cx; ref2.y=cy+10; line(dframe, ref1, ref2, CV_RGB(255,255,0), 1, 8);
        }


    }

    //    namedWindow("Threshold");

    //    imshow("Threshold", outframe);

    return 0;
}


void Image_Processing::doAutoCalibration()
{

    cout << "Image_Processing::doAutoCalibration() is running222...111" << endl;
    find_blob();


}

void Image_Processing::doUndistort(bool flag)
{
    cout << "Image_Processing::doUndistort() is running..." << endl;
    undistort_flag = flag;
}

//Add by Yang Lei 2010.11.23


int Image_Processing::calimain()
{
    using namespace cv;
    //===========================================
    const int number_of_boards = 4; //number of boards
    const int board_dt = 20; //Wait 20 frames per chessboard view
    const int board_w = 4;
    const int board_h = 4;
    static int number_of_corners = board_w * board_h;

    static Size board_size(board_w, board_h);

    //    VideoCapture capture(0);
    //    if (!capture.isOpened())
    //        return -1;
    //    cvNamedWindow("Calibration");

    //ALLOCATE STORAGE
    static Mat object_points(number_of_boards*number_of_corners, 3, CV_32FC1);
    static Mat image_points(number_of_boards*number_of_corners, 2, CV_32FC1);
    static Mat point_counts(number_of_boards, 1, CV_32SC1);
    //static Mat intrinsic_matrix(3, 3, CV_32FC1);
    //static Mat distortion_coeffs(5, 1, CV_32FC1);

    //static Point corners(number_of_corners);
    static vector<Point2f> corners(number_of_corners);
    static int boards_captured = 0;
    static int step, frame = 0;

    //===========================================
    //    Step 1. Capture enough boards         =
    //===========================================
    Mat gray_image(cframe.size(), 8, 1);

    // Capture corner views loop until we have got number_of_boards
    // sucessful captures (All corners on the board are found)
    //
    int speedcounter = 0;
    while (boards_captured < number_of_boards)
    {
        waitKey(5); //yield CPU to other threads;
        //Draw corners
        caldframe = cframe;
        //Print text on output image frame
        char text[50];
        if (speedcounter > 60)
        {speedcounter = 1;}
        sprintf(text, "%d/%d boards captured. %d", boards_captured, number_of_boards, speedcounter++);
        putText(caldframe, text, Point(20,30), FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0),
                2, 5, false);

        //Skip every board_dt frames to allow user to move chessboard
        if(0 == frame++ % board_dt)
        {
            bool found = false;
            //Find chessboard corners
            found = findChessboardCorners(
                    cframe, board_size, corners,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
                    );


            //Get Subpixel accuracy on those corners
            cvtColor(cframe, gray_image, CV_BGR2GRAY);
            Size winSize(11,11);
            Size zeroZone(-1,-1);
            cornerSubPix(gray_image, corners,
                         winSize, zeroZone,
                         TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1)
                         );
            //Draw chessboard
            Mat cornerMat = Mat(corners);
            if((cornerMat.cols == 0) || (cornerMat.rows == 0))
            {
                cout << "No Points found!" << endl;
            }
            else
            {
                drawChessboardCorners(caldframe, board_size, cornerMat, found);
            }

            //ShowImage("Calibration", image);
            //If we got a good board, add it to our data
            if(found)
            {
                step = boards_captured * number_of_corners;
                for (int i = step, j=0; j<number_of_corners; ++i, ++j)
                {
                    image_points.at<float>(i,0) = corners[j].x;
                    image_points.at<float>(i,1) = corners[j].y;
                    object_points.at<float>(i,0) = j/board_w;
                    object_points.at<float>(i,1) = j%board_w;
                    object_points.at<float>(i,2) = 0.0f;
                }
                point_counts.at<int>(boards_captured,0) = number_of_corners;
                boards_captured++;
                cout << boards_captured << " of " << number_of_boards << " boadrs captured!" << endl;
            }
        } //end skip board_dt between chessboard capture

    }//end of collection while loop

    //Allocate Matrices according to how many chessboards found
    //==========================================
    //  Step 2. Data format exchange           =
    //==========================================
    //At this point we have all of the chessboard corners we need.
    //Init the intrinsic matrix such that the two focal length
    //have a ratio of 1.0

    //intrinsic_matrix.at<float>(0,0) = 1.0f;
    //intrinsic_matrix.at<float>(1,1) = 1.0f;

    vector< vector <Point3f> > obj(boards_captured);

    for(uint j = 0, i=0; i<obj.size();++i) {

        obj[i].resize(number_of_corners);

        for(int k=0; k<number_of_corners; ++k ) {
            obj[i][k].x = object_points.at<float>(k+j,0);
            obj[i][k].y = object_points.at<float>(k+j,1);
            obj[i][k].z = object_points.at<float>(k+j,2);
        }
        j+=(number_of_corners);
    }

    vector< vector<Point2f> > img(boards_captured);
    for(uint j = 0, i=0; i<img.size();++i) {

        img[i].resize(number_of_corners);

        for(int k=0; k<number_of_corners; ++k ) {
            img[i][k].x = image_points.at<float>(k+j,0);
            img[i][k].y = image_points.at<float>(k+j,1);
        }
        j+=(number_of_corners);
    }

    //==========================================
    //  Step 3. Do calibration                 =
    //==========================================
    vector<Mat> rvecs, tvecs;
    calibrateCamera(obj,
                    img,
                    cframe.size(),
                    intrinsic_matrix,
                    distortion_coeffs,
                    rvecs, //rvect
                    tvecs, //tvect
                    0);


    cout << "Camera calibration done!" << endl;
    Mat_To_XML();
    return 0;

}
//============================================
//                XML                        =
//============================================
bool Image_Processing::XML_To_Mat()

{
     FileStorage fs ("CameraMatrix.yml", FileStorage::READ);//

     if (fs.isOpened())
     {
        fs["Perspective Matrix"] >> intrinsic_matrix;
        fs["Distortion Matrix"] >> distortion_coeffs;
        cout << "Camera calibration parameters loaded from CameraMatrix.yml" << endl;
        QMessageBox::information(0, "XML_To_MAT()", "Camera calibration parameters are loaded from <CameraMatrix.yml>.");
        return 1;
     }
     else
     {
       return 0;
     }

    //insert here...
}


int Image_Processing::Mat_To_XML()
{
    QDomDocument doc( "CaliParaXML" );
    QDomNode root = doc.createElement( "calipara" );
    doc.appendChild( root );

    //Get perspective parameters:
    fx = intrinsic_matrix.at<double>(0,0);
    fy = intrinsic_matrix.at<double>(1,1);
    cx = intrinsic_matrix.at<double>(0,2);
    cy = intrinsic_matrix.at<double>(1,2);
    //Get distortion coefficients:
    //distCoeffs – The output 5x1 or 1x5 vector of distortion coefficients (k_1, k_2, p_1, p_2[, k_3])
    k1 = distortion_coeffs.at<double>(0,0);
    k2 = distortion_coeffs.at<double>(0,1);
    p1 = distortion_coeffs.at<double>(0,2); //pay attention to the order!
    p2 = distortion_coeffs.at<double>(0,3);
    k3 = distortion_coeffs.at<double>(0,4);

    QDomElement perspective = doc.createElement("Perspective Parameters" );
    perspective.setAttribute("Fx", fx);
    perspective.setAttribute("Fy", fy);
    perspective.setAttribute("Cx", cx);
    perspective.setAttribute("Cy", cy);
    root.appendChild( perspective );

    QDomElement distortion = doc.createElement("Distortion Parameters" );
    distortion.setAttribute("K1", k1);
    distortion.setAttribute("K2", k2);
    distortion.setAttribute("K3", k3);
    distortion.setAttribute("P1", p1);
    distortion.setAttribute("P2", p2);
    root.appendChild( distortion );

    QFile file( "CameraPara.xml" );
    if( !file.open(QIODevice::WriteOnly) )
        return -1;

    QTextStream ts( &file );
    ts << doc.toString();

    file.close();

    //==============YML========================
    FileStorage fs("CameraMatrix.yml", FileStorage::WRITE);
    fs << "Perspective Matrix" << intrinsic_matrix;
    fs << "Distortion Matrix" << distortion_coeffs;

    return 0;
}


void Image_Processing::pEstimate()
{

#ifdef estP3P
    // no armadillo structures outside of P3P, that was the whole point
    // of creating a P3P class instead of simply using the PoseLM directly!
    vector< Point2i > newImgPts(ImagePoints.size());
    vector< Point3f > estPts(ImagePoints.size());

    // randomize newImgPts outside of P3P
    static int mynumberx = 0;
    static int mynumbery = 0;

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
    for (size_t i=0;i<newImgPts.size();++i)
    {
        newImgPts[i].x = ImagePoints.at(i).x + mynumberx;
        newImgPts[i].y = ImagePoints.at(i).y + mynumbery;

        cout << "\t\t" << newImgPts[i].x << "," << newImgPts[i].y << endl;
    }

    // set the new image points
    q->setImagePoints(newImgPts);

    // run the estimator
    q->runEstimator();

    // return result
    estPts = q->getPose();

    // print result for testing
    //q->printResults();

    // register the object points with the estimated points
    transMat = q->getRegister(ObjectPoints,estPts);

    circle(dframe,newImgPts.at(0),15,CV_RGB(255,0,0));
    circle(dframe,newImgPts.at(1),15,CV_RGB(255,0,0));
    circle(dframe,newImgPts.at(2),15,CV_RGB(255,0,0));

    cout << "Final Transformation: " << endl;
    for ( int i=0;i<4;++i)
    {
        for (int j=0;j<4;++j)
            cout << transMat(i,j) << "\t";
        cout << endl;
    }
    cout << endl;
#endif

#ifdef estPOSIT
    p->runEstimator(srcImagePoints);
    srcImagePoints = p->setImagePoints(srcImagePoints);
    cout << srcImagePoints[0].x << " " << srcImagePoints[0].y << endl;
    CvVect32f translation_vector = p->getTvec();
    CvMatr32f rotation_matrix = p->getRmat();
    p->printResults();

    //Project the model points with the estimated pose
    vector<Point2f> projectedPoints;
    for ( size_t  p=0; p<modelPoints.size(); p++ )
    {
        Point3f point3D;
        point3D = Point3f((rotation_matrix[0] * modelPoints[p].x +
                           rotation_matrix[1] * modelPoints[p].y +
                           rotation_matrix[2] * modelPoints[p].z +
                           translation_vector[0]),
                          (rotation_matrix[3] * modelPoints[p].x +
                           rotation_matrix[4] * modelPoints[p].y +
                           rotation_matrix[5] * modelPoints[p].z +
                           translation_vector[1]),
                          (rotation_matrix[6] * modelPoints[p].x +
                           rotation_matrix[7] * modelPoints[p].y +
                           rotation_matrix[8] * modelPoints[p].z +
                           translation_vector[2]));
        Point2f point2D = Point2f( 0.0, 0.0 );
        if ( point3D.z != 0 )
        {
            point2D.x = fl * point3D.x / point3D.z;
            point2D.y = fl * point3D.y / point3D.z;
        }
        projectedPoints.push_back( point2D );
    }

    //Draw the source image points
    int centreX = static_cast<int>( cframe.cols * 0.5 );
    int centreY = static_cast<int>( cframe.rows * 0.5 );

    if(draw_pose) {
        for ( size_t p=0; p<modelPoints.size(); p++ )
            circle( cframe, Point( centreX + (int)srcImagePoints[p].x, centreY - (int)srcImagePoints[p].y ), 8, CV_RGB(255,0,0 ) );

        //Draw the axes
        line( cframe, Point( centreX + (int)projectedPoints[0].x, centreY - (int)projectedPoints[0].y ),
              cvPoint( centreX + (int)projectedPoints[1].x, centreY - (int)projectedPoints[1].y ), CV_RGB(  0, 0, 255 ), 2 );
        line( cframe, Point( centreX + (int)projectedPoints[0].x, centreY - (int)projectedPoints[0].y ),
              cvPoint( centreX + (int)projectedPoints[2].x, centreY - (int)projectedPoints[2].y ), CV_RGB( 255, 0, 0 ), 2 );
        line( cframe, Point( centreX + (int)projectedPoints[0].x, centreY - (int)projectedPoints[0].y ),
              cvPoint( centreX + (int)projectedPoints[3].x, centreY - (int)projectedPoints[3].y ), CV_RGB( 0, 255, 0 ), 2 );
    }
    //Draw the projected model points
    cout << "\n-.- ESTIMATED IMAGE POINTS -.-\n";
    for ( size_t p=0; p<projectedPoints.size(); p++ )
    {
        if(draw_pose)
            circle( cframe, cvPoint( centreX + (int)projectedPoints[p].x, centreY - (int)projectedPoints[p].y ), 3, CV_RGB(255,255,255 ), -1 );
        cout << projectedPoints[p].x << ", " << projectedPoints[p].y << " \n";
    }

    Mat srcPoints(4,2,CV_32F);
    for (size_t i=0;i<4;++i)
    {
        srcPoints.at<float>(i,0) = srcImagePoints[i].x;
        srcPoints.at<float>(i,1) = srcImagePoints[i].y;
    }
    cout << endl;

    int method = 0;
    double ransaceReprojThreshold = 10;
    Mat H(3,3,CV_32F);
    H = findHomography(srcPoints, srcPoints, method, ransaceReprojThreshold);

    cout<< "Homography Matrix:" << endl;
    for (int i=0; i<H.size().height; i++)
        cout << H.at<float>(i,0) << " | " << H.at<float>(i,1) << " | " << H.at<float>(i,2) << endl;
    cout << endl;

#endif

}

/*
//read in dframe, display on cframe, use iframe for calculations
void Image_Processing::FindROI()
{
    //have "load" button to load previously generated calibration
    //upon button click, save current frame, analyze for ROI, display ROI on live feed
    //tell user to keep end effectors within box and to rotate through all possible positions
    //click next button to begin saving contours, click same button to finish
    //prompt user to save calibration, recalibrate, or cancel
    // user will then have to load previously generated calibration
    //display loaded calibration name for each instrument

    vector < Mat > channelsP;
    vector < vector< Point > > contours;

    Mat iframe, bframe;

    // this will be the first frame grabbed from the capture to set up a reference
    // it will be created upon first click of "new tool - get initial frame"
    // this will be used to create our capture rectangle

    Size dframe_sizeP = dframe.size(); //frame sizes

    iframe = dframe;

    //int middle = cframe_size.width/2; //find middle of frame
    int middle = 475; //specified since capture isn't centered

    //draw line through middle of frame for visual check
    if(draw_features)
        line( cframe, Point(middle,0), Point(middle,dframe_sizeP.height ), Scalar(0,0,255), 2, CV_AA );

    cvtColor(iframe, bframe, CV_RGB2YCrCb ); //convert to grayscale or HLS
    split(bframe, channelsP);
    GaussianBlur(channelsP[2], gframe, Size (5,5), 4.0, 4.0 , 1);
    threshold(gframe, tframe, 100, 255, THRESH_BINARY_INV | THRESH_OTSU ); //needs a SINGLE CHANNEL

    // Contours
    findContours(tframe,contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
    Scalar color( 255, 0, 0 );

    if(draw_features)
        drawContours(cframe, contours, -1, color, 2, 8);


    // Classify
    int Num_of_Contours = contours.size();

    int closest_right = dframe_sizeP.width;
    int closest_left = 0;
    int max_y = 0;

    for (int i=0; i < Num_of_Contours; ++i)
    {
        int min_x = dframe_sizeP.width;
        int max_x = 0;

        int contour_length = contours[i].size();
        for (int j=0; j < contour_length; ++j)
        {
            min_x = std::min(min_x,contours[i][j].x);
            max_x = std::max(max_x,contours[i][j].x);
            max_y = std::max(max_y,contours[i][j].y); //Find y closest to bottom
        }

        //extremes
        if (min_x > middle)
        {
            closest_right = std::min(closest_right,min_x);
        }

        if (max_x < middle)
        {
            closest_left = std::max(closest_left,max_x);
        }

    }

    if(draw_features) {
        //draws lines representing the closest points on each contour to the middle
        //line( dframe, Point(closest_right,0), Point(closest_right,cframe_size.height ), Scalar(0,255,0), 2, CV_AA );
        //line( dframe, Point(closest_left,0), Point(closest_left,cframe_size.height ), Scalar(255,0,0), 2, CV_AA );
        //line( dframe, Point(0,max_y), Point(cframe_size.width,max_y ), Scalar(255,255,0), 2, CV_AA );

    }

    Rect roiL( x_offset, y_offset, 50, 50 );
    Rect roiR( closest_right-50, max_y-200, 200, 200 );

    Mat dframe_roiL = dframe(roiL);
    Mat dframe_roiR = dframe(roiR);

}
*/

void Image_Processing::findROI()
{
    // region of interest point values
    // Could add slider to adjust x and y instead of
    // trying to detect instruments
    point_Left_x = 150; // left side of roi
    point_Left_y = 175; // top of roi - decreasing moves roi up
    offset_Left_x = 100; // width to the right
    offset_Left_y = 100; // height going down

    point_Right_x = 390; // left side of roi
    point_Right_y = 175; // top of roi - decreasing moves roi up
    offset_Right_x = 100; // width to the right
    offset_Right_y = 100; // height going down

    // Specify regions of interest
    roiL.x = point_Left_x;
    roiL.y = point_Left_y;
    roiL.width = offset_Left_x;
    roiL.height = offset_Left_y;

    roiR.x = point_Right_x;
    roiR.y = point_Right_y;
    roiR.width = offset_Right_x;
    roiR.height = offset_Right_y;
}

void Image_Processing::SaveContours()
{
    // needed to specify offset for drawing
    vector<Vec4i> hierarchy;

    // Left Side Specifics
    vector < Mat > channelsL;
    vector < vector< Point > > contoursL;
    vector < vector< Point > > Left_Contour;

    Mat iframeL, bframeL, gframeL, tframeL;

    uframe_roiL = uframe(roiL);
    iframeL = uframe_roiL;

    // Left side - find contours
    cvtColor(iframeL, bframeL, CV_RGB2YCrCb ); //convert to grayscale or HLS
    split(bframeL, channelsL);
    GaussianBlur(channelsL[2], gframeL, Size (5,5), 4.0, 4.0 , 1);
    threshold(gframeL, tframeL, 100, 255, THRESH_BINARY_INV | THRESH_OTSU ); //needs a SINGLE CHANNEL
    findContours(tframeL,contoursL, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

    // Find largest contour ---------------------------not working
    int Num_of_Left_Contours = contoursL.size();
    unsigned int size_L = 0;
    for (int i=0; i < Num_of_Left_Contours; ++i)
    {
        if (contoursL[i].size() >= size_L)
        {
            Left_Contour[0] = contoursL[i];
            size_L = Left_Contour.size();
        }
    }

    // counts idx for each saved contour
    static int contour_count = 0;

    // saves left contours to hashL
    hashL.insert(contour_count,Left_Contour.at(0)); // ---------------------------

    // Draw contours for visual check
    Scalar colorL( 255, 0, 0 );
    if(draw_features)
        rectangle(cframe, roiL, Scalar(0,0,255), 1, 8); // Boundary of roiL
        drawContours(cframe, Left_Contour, -1, colorL, 2, 8, hierarchy, 2, Point(point_Left_x,point_Left_y));

    // Right Side Specifics
    vector < Mat > channelsR;
    vector < vector< Point > > contoursR;

    Mat iframeR, bframeR, gframeR, tframeR;

    uframe_roiR = uframe(roiR);
    iframeR = uframe_roiR;

    // Right side - find contours
    cvtColor(iframeR, bframeR, CV_RGB2YCrCb ); //convert to grayscale or HLS
    split(bframeR, channelsR);
    GaussianBlur(channelsR[2], gframeR, Size (5,5), 4.0, 4.0 , 1);
    threshold(gframeR, tframeR, 100, 255, THRESH_BINARY_INV | THRESH_OTSU ); //needs a SINGLE CHANNEL
    findContours(tframeR,contoursR, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

    // saves right contours to hashR
    //hashR.insert(contour_count,contoursR.at(0));

    // Draw contours
    Scalar colorR( 0, 255, 0 );
    if(draw_features)
        rectangle(cframe, roiR, Scalar(0,0,255), 1, 8); // Boundary of roiR
        drawContours(cframe, contoursR, -1, colorR, 2, 8, hierarchy, 2, Point(point_Right_x,point_Right_y));

    //increment for next contour
    contour_count++;
}

void Image_Processing::AddPoints()
{
    // Draw each hash idx contour
    // collect mouse input
    // save to new hash file
}

void Image_Processing::MatchContours()
{
    // run findroi
    // compare roi to each index hash
    // if match, output idx points of other hash file
    // if no match, skip
}






