#include "debug_visualizer.h"
#include <sstream>
#include <iomanip>

DebugVisualizer::DebugVisualizer() {}

void DebugVisualizer::handleKey(int delayMs) {
    int key = cv::waitKey(delayMs) & 0xFF;
    if (key == 'q' || key == 27) 
        shouldQuit_ = true;
}

void DebugVisualizer::render(const cv::Mat& originalFrame,
                              const cv::Mat& /*binaryMask*/,
                              const std::vector<ImageArmor>& armors,
                              const KalmanState6D& kalmanState,
                              const GimbalCommand& gimbalCmd,
                              double fps,
                              double distance,
                              const cv::Point3d& innovation,
                              const cv::Point3d& /*measured*/) {

    cv::Mat detImg = originalFrame.clone();
    drawDetectionOverlay(detImg, armors);
    drawTargetInfo(detImg, kalmanState, gimbalCmd, fps, distance, innovation);
    cv::imshow("Detection", detImg);
}

void DebugVisualizer::drawDetectionOverlay(cv::Mat& img,
                                            const std::vector<ImageArmor>& armors) {
    for (const auto& armor : armors) {
        if (!armor.valid || armor.corners.size() != 4) continue;

        std::vector<cv::Point> pts;
        for (const auto& c : armor.corners)
            pts.push_back(cv::Point(c.x, c.y));
        cv::polylines(img, pts, true, cv::Scalar(0, 255, 0), 2);

        // 画对角线
        cv::line(img, armor.corners[0], armor.corners[2],
                 cv::Scalar(0, 255, 255), 1);
        cv::line(img, armor.corners[1], armor.corners[3],
                 cv::Scalar(0, 255, 255), 1);

        // 画中心点
        cv::circle(img, armor.center, 4, cv::Scalar(0, 0, 255), -1);

        // 画角点
        for (size_t i = 0; i < 4; i++) {
            cv::circle(img, armor.corners[i], 3,
                       cv::Scalar(255, 0, 0), -1);
        }
    }
}

void DebugVisualizer::drawTargetInfo(cv::Mat& img,
                                      const KalmanState6D& state,
                                      const GimbalCommand& cmd,
                                      double fps,
                                      double distance,
                                      const cv::Point3d& innovation) {
    int baseY = 30;
    int lineH = 22;
    cv::Scalar col(0, 255, 255);
    cv::Scalar colGood(0, 255, 0);
    cv::Scalar colWarn(0, 165, 255);

    auto putText = [&](const std::string& text, const cv::Scalar& c) {
        cv::putText(img, text, cv::Point(10, baseY),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, c, 1);
        baseY += lineH;
    };

    std::ostringstream ss;
    ss << "FPS: " << std::fixed << std::setprecision(1) << fps;
    putText(ss.str(), col);

    ss.str(""); ss << "Dist: " << std::setprecision(2) << distance << "m";
    putText(ss.str(), col);

    if (state.valid) {
        ss.str(""); ss << "Pos: (" << std::setprecision(2)
                       << state.x << ", " << state.y << ", " << state.z << ")";
        putText(ss.str(), col);
        ss.str(""); ss << "Vel: (" << std::setprecision(2)
                       << state.vx << ", " << state.vy << ", " << state.vz << ")";
        putText(ss.str(), col);

        double innovNorm = std::sqrt(innovation.x * innovation.x +
                                     innovation.y * innovation.y +
                                     innovation.z * innovation.z);
        ss.str(""); ss << "Innovation: " << std::setprecision(3) << innovNorm << "m"
                       << " (dx=" << std::setprecision(3) << innovation.x
                       << " dy=" << innovation.y
                       << " dz=" << innovation.z << ")";
        cv::Scalar innovColor = (innovNorm < 0.1) ? colGood : colWarn;
        putText(ss.str(), innovColor);
    }

    if (cmd.target_locked) {
        double yawDeg   = cmd.yaw   * 180.0 / CV_PI;
        double pitchDeg = cmd.pitch * 180.0 / CV_PI;
        ss.str(""); ss << "Yaw: " << std::setprecision(2) << yawDeg << "deg"
                       << "  Pitch: " << pitchDeg << "deg";
        putText(ss.str(), col);
        putText("TARGET LOCKED", colGood);
    } else {
        putText("SEARCHING...", col);
    }
}
