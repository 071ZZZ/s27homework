#pragma once

#include "data_types.h"
#include <opencv2/opencv.hpp>
#include <vector>

struct LightBar {
    cv::RotatedRect rect;
    cv::Point2f center;
    cv::Point2f top, bottom;       // 灯条上下端点
    cv::Point2f top2bottom;        // 灯条方向向量 (bottom - top)
    float length = 0;
    float width  = 0;
    float angle  = 0;              // top2bottom 方向角 (rad)
    float angle_error = 0;         // 偏离竖直的角度 (rad)
    float ratio = 0;               // length / width
    int id = -1;                   // 灯条ID，去重用
    cv::Point2f pts[4];
};

struct ArmorCandidate {
    LightBar left, right;
    float ratio;                   // |left2right| / max(left.length, right.length)
    float side_ratio;              // max(left.length, right.length) / min(...)
    float rectangular_error;       // 矩形度误差 (rad)
    bool duplicated = false;
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

    // 灯条筛选参数
    double minContourArea_        = 4.0;
    double maxContourAreaRatio_   = 0.02;   // 相对于图像面积
    float  lightBarMinLength_     = 5.0f;
    float  lightBarMaxLengthRatio_= 0.45f;  // 相对于图像高度
    float  lightBarMaxAngleError_ = 0.8727f; // 角度误差上限 rad (约50°)
    float  lightBarMinRatio_      = 1.3f;
    float  lightBarMaxRatio_      = 20.0f;

    // 配对约束参数
    float pairMaxSideRatio_       = 2.4f;   // 长灯条/短灯条上限
    float pairMaxAngleDiff_       = 28.0f;  // 两灯条角度差上限 (degree)
    float pairMaxYOffsetRatio_    = 1.0f;   // dy < avgLength * ratio
    float pairMinArmorRatio_      = 0.5f;   // armor ratio 下限
    float pairMaxArmorRatio_      = 3.5f;   // armor ratio 上限
    float pairMaxArmorAngle_      = 38.0f;  // 两灯条连线与水平的夹角上限 (degree)
    float pairMaxDxRatio_         = 0.35f;  // dx < frameWidth * ratio
    float pairMinOverlapRatio_    = 0.3f;   // y-投影重叠 / 较长灯条长度
    float pairMaxWidthRatio_      = 2.5f;   // 两灯条宽度比上限
    float pairMaxRectError_       = 0.15f;  // 矩形度误差上限 (rad, 约8.6°)

    std::vector<LightBar> lastLightBars_;
};
