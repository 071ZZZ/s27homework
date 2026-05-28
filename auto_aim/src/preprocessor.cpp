#include "preprocessor.h"

Preprocessor::Preprocessor() {}

void Preprocessor::setRedRangeLow(int hMin, int sMin, int vMin,
                                   int hMax, int sMax, int vMax) {
    hLow1_ = hMin; sLow1_ = sMin; vLow1_ = vMin;
    hHigh1_ = hMax; sHigh1_ = sMax; vHigh1_ = vMax;
}

void Preprocessor::setRedRangeHigh(int hMin, int sMin, int vMin,
                                    int hMax, int sMax, int vMax) {
    hLow2_ = hMin; sLow2_ = sMin; vLow2_ = vMin;
    hHigh2_ = hMax; sHigh2_ = sMax; vHigh2_ = vMax;
}

cv::Mat Preprocessor::process(const cv::Mat& bgrFrame) {
    cv::Mat hsv;
    cv::cvtColor(bgrFrame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask1, mask2;
    cv::inRange(hsv,
                cv::Scalar(hLow1_, sLow1_, vLow1_),
                cv::Scalar(hHigh1_, sHigh1_, vHigh1_),
                mask1);
    cv::inRange(hsv,
                cv::Scalar(hLow2_, sLow2_, vLow2_),
                cv::Scalar(hHigh2_, sHigh2_, vHigh2_),
                mask2);
    cv::Mat hsvMask = mask1 | mask2; 

    std::vector<cv::Mat> bgr;
    cv::split(bgrFrame, bgr);
    cv::Mat R = bgr[2], G = bgr[1], B = bgr[0];

    cv::Mat rgDiff, rbDiff;
    cv::subtract(R, G, rgDiff);
    cv::subtract(R, B, rbDiff);

    cv::Mat rgMask, rbMask, rMask;
    cv::threshold(rgDiff, rgMask, rgDiffThresh_, 255, cv::THRESH_BINARY);
    cv::threshold(rbDiff, rbMask, rbDiffThresh_, 255, cv::THRESH_BINARY);
    cv::threshold(R, rMask, rChanThresh_, 255, cv::THRESH_BINARY);

    cv::Mat mask = hsvMask & rgMask & rbMask & rMask;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    cv::Mat dilateKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::dilate(mask, mask, dilateKernel);

    lastRawMask_ = mask;

    cv::Mat roi = cv::Mat::zeros(mask.size(), CV_8UC1);
    int y0 = static_cast<int>(mask.rows * 0.25);
    cv::rectangle(roi, cv::Rect(0, y0, mask.cols, mask.rows - y0),
                  cv::Scalar(255), -1);

    return mask & roi;
}
