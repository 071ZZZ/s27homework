#include "preprocessor.h"

Preprocessor::Preprocessor() {}

cv::Mat Preprocessor::process(const cv::Mat& bgrFrame) {
    // 1. 转灰度
    cv::Mat gray;
    cv::cvtColor(bgrFrame, gray, cv::COLOR_BGR2GRAY);

    // 2. 全局阈值二值化
    //    灯条区域为高亮白色，阈值取较高值滤除大部分背景
    cv::Mat mask;
    cv::threshold(gray, mask, grayThreshold_, 255, cv::THRESH_BINARY);
    cv::imshow("binary", mask);

    // 3. 轻度形态学：对称核，仅用于消除孤立噪点
    if (useMorphology_) {
        cv::Mat openKernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(openKernelSize_, openKernelSize_));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, openKernel);

        cv::Mat dilateKernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(dilateKernelSize_, dilateKernelSize_));
        cv::dilate(mask, mask, dilateKernel);
    }

    lastRawMask_ = mask;
    return mask;
}

LightColor Preprocessor::getColor(const cv::Mat& bgr,
                                   const std::vector<cv::Point>& contour) {
    int red_sum = 0, blue_sum = 0;
    for (const auto& point : contour) {
        const auto& pixel = bgr.at<cv::Vec3b>(point);
        blue_sum += pixel[0];  // BGR: channel 0 = B
        red_sum  += pixel[2];  // BGR: channel 2 = R
    }
    if (blue_sum > red_sum) return LightColor::BLUE;
    if (red_sum > blue_sum)  return LightColor::RED;
    return LightColor::UNKNOWN;
}
