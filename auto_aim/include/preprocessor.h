#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

enum class LightColor { RED, BLUE, UNKNOWN };

class Preprocessor {
public:
    Preprocessor();

    cv::Mat process(const cv::Mat& bgrFrame);

    // 参考 sp_vision_25 detector.cpp:279 get_color
    // 按轮廓像素和判断颜色（BGR 图 + 轮廓点 → 累加 R/B 通道比较）
    static LightColor getColor(const cv::Mat& bgr,
                                const std::vector<cv::Point>& contour);

    cv::Mat getLastRawMask() const { return lastRawMask_; }

private:
    // 参考 sp_vision_25: 灰度全局阈值，灯条区域亮度高
    int grayThreshold_ = 180;

    // 轻度形态学（对称核），仅用于去除孤立噪点
    bool useMorphology_ = true;
    int openKernelSize_   = 2;
    int dilateKernelSize_ = 2;

    cv::Mat lastRawMask_;
};
