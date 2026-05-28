#pragma once

#include "preprocessor.h"
#include "armor_detector.h"
#include "pnp.h"
#include "kalman_filter_3d.h"
#include "coordinate_transformer.h"
#include "debug_visualizer.h"

#include <opencv2/opencv.hpp>
#include <string>

class AutoAimSystem {
public:
    AutoAimSystem();

    bool initialize(const std::string& videoPath,
                    const std::string& calibPath);

    void setBulletSpeed(double speed) { bulletSpeed_ = speed; }

    void run();

    bool processFrame(const cv::Mat& frame, GimbalCommand& output);

private:
    Preprocessor          preprocessor_;
    ArmorDetector         armorDetector_;
    PnP                   pnp_;
    KalmanFilter3D        kalmanFilter_;
    CoordinateTransformer transformer_;
    DebugVisualizer       visualizer_;

    cv::VideoCapture capture_;
    double startTime_ = 0;
    int    frameCount_ = 0;
    double bulletSpeed_ = 18.0;

    // 存储最近一次结果用于可视化
    cv::Mat                   lastMask_;
    cv::Mat                   lastFrame_;
    std::vector<ImageArmor>   lastArmors_;
    KalmanState6D             lastKalmanState_;
    GimbalCommand             lastCmd_;
    double                    lastFps_   = 0;
    double                    lastDist_  = 0;

    // 用于验证滤波效果的残差统计
    cv::Point3d               lastMeasured_;     // PnP原始测量值
    cv::Point3d               lastInnovation_;   // 残差 = 测量 - 预测
    double                    lastFlyTime_ = 0;  // 子弹飞行时间

    double getTimestamp() const;
};
