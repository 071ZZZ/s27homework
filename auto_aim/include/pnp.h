#pragma once

#include "data_types.h"
#include <opencv2/opencv.hpp>

class PnP {
public:
    PnP();

    bool loadCalibration(const std::string& yamlPath);
    bool estimate(const std::vector<cv::Point2f>& imageCorners, Target3D& target);

    static std::vector<cv::Point3f> getArmorModelPoints();

private:
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    static constexpr double kArmorWidth  = 0.135;
    static constexpr double kArmorHeight = 0.125;

    std::vector<cv::Point3f> modelPoints_;
};
