#pragma once

#include "data_types.h"
#include <opencv2/opencv.hpp>
#include <vector>

struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center;
    float length = 0;
    float width  = 0;
    float angle  = 0;

    // 4个角点 (用于后续PnP)
    cv::Point2f pts[4];
};

struct ArmorCandidate {
    LightBar left;
    LightBar right;
    double score = 0;
};

class ArmorDetector {
public:
    ArmorDetector();

    std::vector<ImageArmor> detect(const cv::Mat& binaryMask, const cv::Size& frameSize);

    const std::vector<LightBar>& getLastLightBars() const { return lastLightBars_; }

private:
    bool isLightBar(const std::vector<cv::Point>& contour, LightBar& lb,
                    const cv::Size& frameSize);

    bool tryPair(const LightBar& a, const LightBar& b, ArmorCandidate& armor,
                 const cv::Size& frameSize);

    static float normalizeAngle(float angle);
    static float angleDiff(float a, float b);

    // 灯条筛选参数
    double minContourArea_        = 4.0;
    double maxContourAreaRatio_   = 0.02;   // 相对于图像面积
    float  lightBarMinLength_     = 5.0f;
    float  lightBarMaxLengthRatio_= 0.45f;  // 相对于图像高度
    float  lightBarMaxAngle_      = 50.0f;
    float  lightBarMinRatio_      = 1.3f;
    float  lightBarMaxRatio_      = 20.0f;

    // 配对约束参数
    float  pairMaxHeightRatio_    = 2.4f;
    float  pairMaxAngleDiff_      = 28.0f;
    float  pairMaxYOffsetRatio_   = 1.0f;   // dy < avgLength * ratio
    float  pairDistRatioMin_      = 0.5f;
    float  pairDistRatioMax_      = 7.0f;
    float  pairMaxArmorAngle_     = 38.0f;  // 两灯条连线与水平的夹角
    float  pairMaxDxRatio_        = 0.35f;  // dx < frameWidth * ratio

    std::vector<LightBar> lastLightBars_;
};
