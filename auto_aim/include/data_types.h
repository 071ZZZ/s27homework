#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

// 图像空间中的装甲板
struct ImageArmor {
    std::vector<cv::Point2f> corners; // 4角点: TL, TR, BR, BL
    cv::Point2f              center;  // 对角线交点
    bool                     valid = false;
};

// PnP解算得到的3D目标(相机坐标系)
struct Target3D {
    double   x = 0, y = 0, z = 0;
    cv::Mat  rvec;
    cv::Mat  tvec;
    bool     valid = false;
};

// 卡尔曼滤波6状态
struct KalmanState6D {
    double x  = 0, y  = 0, z  = 0;
    double vx = 0, vy = 0, vz = 0;
    bool valid = false;
};

// 云台控制指令
struct GimbalCommand {
    double yaw   = 0;
    double pitch = 0;
    double roll  = 0;
    bool   target_locked = false;
};
