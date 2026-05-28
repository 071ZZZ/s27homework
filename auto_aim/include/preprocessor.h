#pragma once

#include <opencv2/opencv.hpp>

class Preprocessor {
public:
    Preprocessor();

    void setRedRangeLow(int hMin, int sMin, int vMin, int hMax, int sMax, int vMax);
    void setRedRangeHigh(int hMin, int sMin, int vMin, int hMax, int sMax, int vMax);

    cv::Mat process(const cv::Mat& bgrFrame);

    cv::Mat getLastRawMask() const { return lastRawMask_; }

private:
    // 红色范围1 (低H): 0~30
    int hLow1_ = 0,   sLow1_ = 35,  vLow1_ = 55;
    int hHigh1_ = 30, sHigh1_ = 255, vHigh1_ = 255;

    // 红色范围2 (高H): 155~180
    int hLow2_ = 155, sLow2_ = 35,  vLow2_ = 55;
    int hHigh2_ = 180, sHigh2_ = 255, vHigh2_ = 255;

    // 颜色差值阈值
    int rgDiffThresh_  = 12;   // R - G 最小值
    int rbDiffThresh_  = 8;    // R - B 最小值
    int rChanThresh_   = 70;   // R 通道最小值

    cv::Mat lastRawMask_;
};
