#include "kalman_filter_3d.h"

KalmanFilter3D::KalmanFilter3D() {
    // 状态 [px, py, pz, vx, vy, vz]^T
    x_ = cv::Mat::zeros(6, 1, CV_64F);
    P_ = cv::Mat::eye(6, 6, CV_64F);
    I_ = cv::Mat::eye(6, 6, CV_64F);

    // 测量矩阵 [I_3 | 0_3]
    H_ = cv::Mat::zeros(3, 6, CV_64F);
    H_.at<double>(0,0) = 1;
    H_.at<double>(1,1) = 1;
    H_.at<double>(2,2) = 1;

    // 默认噪声参数
    R_ = cv::Mat::zeros(3, 3, CV_64F);
    R_.at<double>(0,0) = 0.0001;
    R_.at<double>(1,1) = 0.0001;
    R_.at<double>(2,2) = 0.001;
}

void KalmanFilter3D::init(const cv::Point3d& position, double timestamp) {
    x_.at<double>(0) = position.x;
    x_.at<double>(1) = position.y;
    x_.at<double>(2) = position.z;
    x_.at<double>(3) = 0.0;  // vx
    x_.at<double>(4) = 0.0;  // vy
    x_.at<double>(5) = 0.0;  // vz

    P_ = cv::Mat::eye(6, 6, CV_64F);
    P_.at<double>(0,0) = 100.0;
    P_.at<double>(1,1) = 100.0;
    P_.at<double>(2,2) = 100.0;
    P_.at<double>(3,3) = 10.0;
    P_.at<double>(4,4) = 10.0;
    P_.at<double>(5,5) = 10.0;

    lastTime_    = timestamp;
    initialized_ = true;
}

void KalmanFilter3D::buildTransition(double dt) {
    F_ = cv::Mat::eye(6, 6, CV_64F);
    F_.at<double>(0,3) = dt;
    F_.at<double>(1,4) = dt;
    F_.at<double>(2,5) = dt;
}

void KalmanFilter3D::buildProcessNoise(double dt, double q) {
    double dt2 = dt * dt;
    double dt3 = dt2 * dt;

    Q_ = cv::Mat::zeros(6, 6, CV_64F);
    double q_dt3_3 = q * dt3 / 3.0;
    double q_dt2_2 = q * dt2 / 2.0;
    double q_dt     = q * dt;

    // X轴块
    Q_.at<double>(0,0) = q_dt3_3;
    Q_.at<double>(0,3) = q_dt2_2;
    Q_.at<double>(3,0) = q_dt2_2;
    Q_.at<double>(3,3) = q_dt;

    // Y轴块
    Q_.at<double>(1,1) = q_dt3_3;
    Q_.at<double>(1,4) = q_dt2_2;
    Q_.at<double>(4,1) = q_dt2_2;
    Q_.at<double>(4,4) = q_dt;

    // Z轴块
    Q_.at<double>(2,2) = q_dt3_3;
    Q_.at<double>(2,5) = q_dt2_2;
    Q_.at<double>(5,2) = q_dt2_2;
    Q_.at<double>(5,5) = q_dt;
}

KalmanState6D KalmanFilter3D::predict(double timestamp) {
    if (!initialized_) {
        KalmanState6D s;
        s.valid = false;
        return s;
    }

    double dt = timestamp - lastTime_;

    if (dt > maxPredictionAge_) {
        reset();
        KalmanState6D s;
        s.valid = false;
        return s;
    }

    if (dt <= 0) dt = 1e-6;

    buildTransition(dt);
    buildProcessNoise(dt, qAccel_);

    // x_pred = F * x
    x_ = F_ * x_;
    // P_pred = F * P * F^T + Q
    P_ = F_ * P_ * F_.t() + Q_;

    lastTime_ = timestamp;

    KalmanState6D state;
    state.x  = x_.at<double>(0);
    state.y  = x_.at<double>(1);
    state.z  = x_.at<double>(2);
    state.vx = x_.at<double>(3);
    state.vy = x_.at<double>(4);
    state.vz = x_.at<double>(5);
    state.valid = true;
    return state;
}

KalmanState6D KalmanFilter3D::predictDelta(double dt) const {
    KalmanState6D state;
    if (!initialized_ || dt <= 0) {
        state = getState();
        return state;
    }

    // 构建转移矩阵：不使用内部 F_ 避免副作用
    cv::Mat F_local = cv::Mat::eye(6, 6, CV_64F);
    F_local.at<double>(0,3) = dt;
    F_local.at<double>(1,4) = dt;
    F_local.at<double>(2,5) = dt;

    cv::Mat x_future = F_local * x_;

    state.x  = x_future.at<double>(0);
    state.y  = x_future.at<double>(1);
    state.z  = x_future.at<double>(2);
    state.vx = x_future.at<double>(3);
    state.vy = x_future.at<double>(4);
    state.vz = x_future.at<double>(5);
    state.valid = true;
    return state;
}

void KalmanFilter3D::update(const cv::Point3d& measurement) {
    if (!initialized_) return;

    // 测量 z (3x1)
    cv::Mat z(3, 1, CV_64F);
    z.at<double>(0) = measurement.x;
    z.at<double>(1) = measurement.y;
    z.at<double>(2) = measurement.z;

    // y = z - H*x
    cv::Mat y = z - H_ * x_;

    // S = H * P * H^T + R
    cv::Mat S = H_ * P_ * H_.t() + R_;

    // K = P * H^T * S^{-1}
    cv::Mat K = P_ * H_.t() * S.inv();

    // x = x + K*y
    x_ = x_ + K * y;

    // P = (I - K*H) * P
    P_ = (I_ - K * H_) * P_;
}

KalmanState6D KalmanFilter3D::getState() const {
    KalmanState6D state;
    state.x  = x_.at<double>(0);
    state.y  = x_.at<double>(1);
    state.z  = x_.at<double>(2);
    state.vx = x_.at<double>(3);
    state.vy = x_.at<double>(4);
    state.vz = x_.at<double>(5);
    state.valid = initialized_;
    return state;
}

void KalmanFilter3D::reset() {
    initialized_ = false;
    x_ = cv::Mat::zeros(6, 1, CV_64F);
    P_ = cv::Mat::eye(6, 6, CV_64F);
}
