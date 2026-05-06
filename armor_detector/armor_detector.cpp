#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

Point2f getIntersection(Point2f A, Point2f B, Point2f C, Point2f D)
{
    float a1 = B.y - A.y;
    float b1 = A.x - B.x;
    float c1 = a1 * A.x + b1 * A.y;

    float a2 = D.y - C.y;
    float b2 = C.x - D.x;
    float c2 = a2 * C.x + b2 * C.y;

    float det = a1 * b2 - a2 * b1;

    if (abs(det) < 1e-5)
        return Point2f(0, 0);

    return Point2f(
        (b2 * c1 - b1 * c2) / det,
        (a1 * c2 - a2 * c1) / det
    );
}

int main()
{
     VideoCapture cap("/home/lqy/s27_homework/src/armors/avi.mp4");

    if (!cap.isOpened())
    {
        cout << "视频打开失败！" << endl;
        return -1;
    }
    Mat img;
    while (true)
    {
        cap >> img;
        if (img.empty()) break;

    Mat img_show = img.clone();

    Mat blur, hsv, mask;
    GaussianBlur(img, blur, Size(5,5), 0);
    cvtColor(blur, hsv, COLOR_BGR2HSV);

    Mat mask1, mask2;
    inRange(hsv, Scalar(0, 50, 50), Scalar(15, 255, 255), mask1);
    inRange(hsv, Scalar(160, 50, 50), Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2; 

    Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);

    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<RotatedRect> lights;

    for (auto& c : contours)
    {
        if (contourArea(c) < 50) continue;

        RotatedRect rect = minAreaRect(c);

        float w = rect.size.width;
        float h = max(rect.size.height, rect.size.width);
        if (h < 10 || h > 100) continue;
        
        float angle = rect.angle;
        if (angle < -90) angle += 180;
        if (abs(angle) > 30) continue;

        float ratio = max(w, h) / min(w, h);

        if (ratio > 2.0 && ratio < 10.0)
        {
            lights.push_back(rect);
        }
    }


    for (int i = 0; i < lights.size(); i++)
    {
        for (int j = i + 1; j < lights.size(); j++)
        {
            RotatedRect l1 = lights[i];
            RotatedRect l2 = lights[j];

            RotatedRect left = l1.center.x < l2.center.x ? l1 : l2;
            RotatedRect right = l1.center.x < l2.center.x ? l2 : l1;

            float h1 = max(left.size.height, left.size.width);
            float h2 = max(right.size.height, right.size.width);

            if (abs(h1 - h2) / max(h1, h2) > 0.3) continue;

            if (abs(left.center.y - right.center.y) > 30) continue;

            float dx = abs(left.center.x - right.center.x);
            if (dx / h1 < 2.0 || dx / h1 > 3.0) continue;

            if (abs(left.angle - right.angle) > 15) continue;

            Point2f pts_l[4], pts_r[4];
            left.points(pts_l);
            right.points(pts_r);

            sort(pts_l, pts_l + 4, [](Point2f a, Point2f b){ return a.y < b.y; });
            sort(pts_r, pts_r + 4, [](Point2f a, Point2f b){ return a.y < b.y; });

            Point2f left_top = (pts_l[0] + pts_l[1]) * 0.5;
            Point2f left_bottom = (pts_l[2] + pts_l[3]) * 0.5;

            Point2f right_top = (pts_r[0] + pts_r[1]) * 0.5;
            Point2f right_bottom = (pts_r[2] + pts_r[3]) * 0.5;

            line(img_show, left_top, right_bottom, Scalar(255,0,255), 2);
            line(img_show, left_bottom, right_top, Scalar(255,0,255), 2);

            Point2f center = getIntersection(left_top, right_bottom,
                                             left_bottom, right_top);

            circle(img_show, center, 6, Scalar(0,0,255), -1);

            Point2f box[4];
            left.points(box);
            for (int k = 0; k < 4; k++)
                line(img_show, box[k], box[(k+1)%4], Scalar(255,255,0), 1);

            right.points(box);
            for (int k = 0; k < 4; k++)
                line(img_show, box[k], box[(k+1)%4], Scalar(255,255,0), 1);
        }
    }

    imshow("img", img);
    imshow("result", img_show);
    if(waitKey(30)=='q')
        break;
}
    cap.release();
    destroyAllWindows();
    return 0;
}