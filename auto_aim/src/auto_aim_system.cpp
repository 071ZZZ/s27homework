#include "auto_aim_system.h"
#include <iostream>
#include <chrono>
#include <iomanip>

AutoAimSystem::AutoAimSystem() {}

bool AutoAimSystem::initialize(const std::string& videoPath,
                                const std::string& calibPath) {
    if (!pnp_.loadCalibration(calibPath)) {
        std::cerr << "Failed to load calibration: " << calibPath << std::endl;
        return false;
    }

    capture_.open(videoPath);
    if (!capture_.isOpened()) {
        std::cerr << "Failed to open video: " << videoPath << std::endl;
        return false;
    }

    startTime_ = static_cast<double>(cv::getTickCount()) / cv::getTickFrequency();

    std::cout << "AutoAimSystem initialized." << std::endl;
    std::cout << "  Video: " << videoPath << std::endl;
    std::cout << "  Calib: " << calibPath << std::endl;
    std::cout << "  Bullet speed: " << bulletSpeed_ << " m/s" << std::endl;
    std::cout << "Press 'q' or ESC to quit." << std::endl;
    return true;
}

double AutoAimSystem::getTimestamp() const {
    return static_cast<double>(cv::getTickCount()) / cv::getTickFrequency() - startTime_;
}

bool AutoAimSystem::processFrame(const cv::Mat& frame, GimbalCommand& output) {
    lastFrame_ = frame;

    lastMask_ = preprocessor_.process(frame);

    lastArmors_ = armorDetector_.detect(lastMask_, frame.size());

    if (!lastArmors_.empty()) {
        // 选最靠近画面中心的装甲板
        cv::Point2f imgCenter(frame.cols * 0.5f, frame.rows * 0.5f);
        ImageArmor* bestArmor = nullptr;
        double bestDist = 1e9;
        for (auto& a : lastArmors_) {
            double d = cv::norm(a.center - imgCenter);
            if (d < bestDist) { bestDist = d; bestArmor = &a; }
        }
        //PnP解算
        if (bestArmor) {
            Target3D target;
            if (pnp_.estimate(bestArmor->corners, target)) {
                cv::Point3d pos(target.x, target.y, target.z);
                lastMeasured_ = pos;
                double ts = getTimestamp();

                //卡尔曼滤波
                if (!kalmanFilter_.isInitialized()) {
                    kalmanFilter_.init(pos, ts);
                    lastInnovation_ = cv::Point3d(0, 0, 0);
                } else {
                    kalmanFilter_.predict(ts);
                    // 计算残差：测量值 - 预测值
                    KalmanState6D predState = kalmanFilter_.getState();
                    lastInnovation_ = cv::Point3d(pos.x - predState.x,
                                                   pos.y - predState.y,
                                                   pos.z - predState.z);
                    kalmanFilter_.update(pos);
                }

                //计算弹道预测
                lastKalmanState_ = kalmanFilter_.getState();

                // 子弹飞行时间
                double dist = std::sqrt(lastKalmanState_.x * lastKalmanState_.x +
                                        lastKalmanState_.y * lastKalmanState_.y +
                                        lastKalmanState_.z * lastKalmanState_.z);
                lastFlyTime_ = dist / bulletSpeed_;

                // 预测子弹到达时的位置
                KalmanState6D predicted = kalmanFilter_.predictDelta(lastFlyTime_);
                cv::Point3d aimPos(predicted.x, predicted.y, predicted.z);

                //坐标变换
                output = transformer_.computeGimbalAngles(aimPos);
                lastCmd_ = output;
                return true;
            }
        }
    }

    // 未检测到装甲板 — 只预测不更新
    if (kalmanFilter_.isInitialized()) {
        double ts = getTimestamp();
        lastKalmanState_ = kalmanFilter_.predict(ts);
        if (lastKalmanState_.valid) {
            // 子弹飞行时间补偿
            double dist = std::sqrt(lastKalmanState_.x * lastKalmanState_.x +
                                    lastKalmanState_.y * lastKalmanState_.y +
                                    lastKalmanState_.z * lastKalmanState_.z);
            double flyTime = dist / bulletSpeed_;
            KalmanState6D predicted = kalmanFilter_.predictDelta(flyTime);
            cv::Point3d aimPos(predicted.x, predicted.y, predicted.z);
            output = transformer_.computeGimbalAngles(aimPos);
            output.target_locked = true;
            lastCmd_ = output;
            return true;
        }
    }

    lastKalmanState_ = KalmanState6D{};
    output.target_locked = false;
    lastCmd_ = output;
    return false;
}

void AutoAimSystem::run() {
    cv::Mat frame;
    while (true) {
        capture_ >> frame;
        if (frame.empty()) break;

        frameCount_++;
        GimbalCommand cmd;
        processFrame(frame, cmd);

        // 计算 FPS 和距离
        double elapsed = getTimestamp();
        lastFps_  = (elapsed > 0) ? frameCount_ / elapsed : 0;
        lastDist_ = lastKalmanState_.valid
            ? std::sqrt(lastKalmanState_.x * lastKalmanState_.x +
                        lastKalmanState_.y * lastKalmanState_.y +
                        lastKalmanState_.z * lastKalmanState_.z) : 0;

        // 控制台输出角度
        if (cmd.target_locked && frameCount_ % 10 == 0) {
            double yawDeg   = cmd.yaw   * 180.0 / CV_PI;
            double pitchDeg = cmd.pitch * 180.0 / CV_PI;
            double innovNorm = std::sqrt(lastInnovation_.x * lastInnovation_.x +
                                         lastInnovation_.y * lastInnovation_.y +
                                         lastInnovation_.z * lastInnovation_.z);
            int nLightBars = armorDetector_.getLastLightBars().size();
            int nArmors    = lastArmors_.size();
            std::cout << "\r[Frame " << frameCount_ << "] "
                      << "Yaw: " << std::fixed << std::setprecision(2) << yawDeg << "deg  "
                      << "Pitch: " << pitchDeg << "deg  "
                      << "Dist: " << std::setprecision(2) << lastDist_ << "m  "
                      << "Innov: " << std::setprecision(3) << innovNorm << "m  "
                      << "FlyTime: " << std::setprecision(0) << lastFlyTime_ * 1000 << "ms  "
                      << "Bars:" << nLightBars << "  "
                      << "Armors:" << nArmors << "  "
                      << "FPS: " << std::setprecision(1) << lastFps_
                      << std::flush;
        } else if (!cmd.target_locked && frameCount_ % 30 == 0) {
            int nLightBars = armorDetector_.getLastLightBars().size();
            std::cout << "\r[Frame " << frameCount_ << "] SEARCHING...  Bars:" << nLightBars
                      << "                      " << std::flush;
        }

        // 用存储的结果做可视化
        visualizer_.render(lastFrame_, lastMask_, lastArmors_,
                           lastKalmanState_, lastCmd_, lastFps_, lastDist_,
                           lastInnovation_, lastMeasured_);
        visualizer_.handleKey(30);

        if (visualizer_.shouldQuit()) break;
    }

    std::cout << "\nDone. Processed " << frameCount_ << " frames." << std::endl;
}
