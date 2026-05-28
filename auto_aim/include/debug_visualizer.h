#pragma once

#include "data_types.h"
#include <opencv2/opencv.hpp>
#include <vector>

class DebugVisualizer {
public:
    DebugVisualizer();

    void render(const cv::Mat& originalFrame,
                const cv::Mat& binaryMask,
                const std::vector<ImageArmor>& armors,
                const KalmanState6D& kalmanState,
                const GimbalCommand& gimbalCmd,
                double fps,
                double distance,
                const cv::Point3d& innovation,
                const cv::Point3d& measured);

    bool shouldQuit() const { return shouldQuit_; }
    void handleKey(int delayMs = 30);

private:
    bool shouldQuit_ = false;

    void drawDetectionOverlay(cv::Mat& img, const std::vector<ImageArmor>& armors);
    void drawTargetInfo(cv::Mat& img, const KalmanState6D& state,
                        const GimbalCommand& cmd, double fps, double distance,
                        const cv::Point3d& innovation);
};
