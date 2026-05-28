#pragma once

#include "data_types.h"
#include <opencv2/opencv.hpp>

class KalmanFilter3D {
public:
    KalmanFilter3D();

    void init(const cv::Point3d& position, double timestamp);

    KalmanState6D predict(double timestamp);

    KalmanState6D predictDelta(double dt) const;

    void update(const cv::Point3d& measurement);

    bool isInitialized() const { return initialized_; }

    KalmanState6D getState() const;

    void reset();

private:
    cv::Mat x_;       // 6x1 state [px,py,pz,vx,vy,vz]
    cv::Mat P_;       // 6x6 covariance
    cv::Mat F_;       // 6x6 state transition (rebuilt per predict)
    cv::Mat H_;       // 3x6 measurement matrix
    cv::Mat Q_;       // 6x6 process noise (rebuilt per predict)
    cv::Mat R_;       // 3x3 measurement noise
    cv::Mat I_;       // 6x6 identity

    bool   initialized_ = false;
    double lastTime_    = 0.0;
    double maxPredictionAge_ = 0.5;
    double qAccel_ = 5.0;

    void buildTransition(double dt);
    void buildProcessNoise(double dt, double q);
};
