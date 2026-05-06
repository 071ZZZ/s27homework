#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int main()
{
    Size boardSize(10, 7);
    float squareSize = 0.1; 

    vector<Point3f> objp;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            objp.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;

    vector<String> images;
    glob("../img/*.jpg", images); 

    if (images.empty()) {
        cout << "没有找到图片！" << endl;
        return -1;
    }

    Size imageSize;

    for (auto& fname : images) {
        Mat img = imread(fname);
        if (img.empty()) continue;

        imageSize = img.size();

        vector<Point2f> corners;
        bool found = findChessboardCorners(img, boardSize, corners);

        if (found) {
            Mat gray;
            cvtColor(img, gray, COLOR_BGR2GRAY);

            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

            imagePoints.push_back(corners);
            objectPoints.push_back(objp);

            drawChessboardCorners(img, boardSize, corners, found);
            imshow("Corners", img);
            waitKey(100);
        }
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;

    calibrateCamera(objectPoints, imagePoints, imageSize,
                    cameraMatrix, distCoeffs, rvecs, tvecs);
    
    double repError = calibrateCamera(objectPoints, imagePoints, imageSize,
                    cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "相机内参矩阵：" << endl << cameraMatrix << endl;
    cout << "畸变系数：" << endl << distCoeffs << endl;
    cout << "重投影误差：" << repError << " 像素" << endl;

    FileStorage fs("../calibration.yaml", FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();

    cout << "标定完成，结果已保存到 calibration.yaml" << endl;

    return 0;
}