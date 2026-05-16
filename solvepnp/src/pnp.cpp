#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int main()
{
    Mat cameraMatrix = (Mat_<double>(3,3) <<
    1567.7907457705167, 0.0, 662.3933648922284, 0.0, 1564.9113082257936, 536.8662848443158, 0.0, 0.0, 1.0);

    Mat distCoeffs = (Mat_<double>(1,5) << 
    -0.0682737005569565, 0.1983544402464456, 0.0016855914452479342, 0.0024125119646311016, 0.0);

    vector<Point3f> objectPoints;
    int rows = 7;
    int cols = 10;
    float squareSize = 0.1;

    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            objectPoints.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    Mat image = imread("/home/lqy/s26_ws/get_intrinsic_camera_calibration/img/2026-04-19_21-43-30_608.jpg");
    if(image.empty())
    {
        cout << "error" << endl;
        return -1;
    }

    vector<Point2f> imagePoints;
    Size patternSize(cols, rows);

    bool found = findChessboardCorners(image, patternSize, imagePoints);

    if(!found)
    {
        cout << "not found" << endl;
        return -1;
    }

    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    cornerSubPix(gray, imagePoints, Size(11,11), Size(-1,-1),
                 TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.01));

    Mat rvec, tvec;

    solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs,
             rvec, tvec, false, SOLVEPNP_ITERATIVE);

    Mat R;
    Rodrigues(rvec, R);

    cout << rvec << endl;
    cout << tvec << endl;
    cout << R << endl;

    vector<Point2f> projectedPoints;
    projectPoints(objectPoints, rvec, tvec,
                  cameraMatrix, distCoeffs, projectedPoints);

    for(size_t i = 0; i < projectedPoints.size(); i++)
    {
        circle(image, projectedPoints[i], 3, Scalar(0,0,255), -1);
    }

    imshow("result", image);
    waitKey(0);

    return 0;
}