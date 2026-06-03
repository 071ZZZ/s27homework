#include "pnp.h"

PnP::PnP() {
    modelPoints_ = getArmorModelPoints();
}

std::vector<cv::Point3f> PnP::getArmorModelPoints() {
    double hw = kArmorWidth  * 0.5;  // 0.0675
    double hh = kArmorHeight * 0.5;  // 0.0625
    return {
        cv::Point3f(-hw, -hh, 0.0),  // TL
        cv::Point3f( hw, -hh, 0.0),  // TR
        cv::Point3f( hw,  hh, 0.0),  // BR
        cv::Point3f(-hw,  hh, 0.0)   // BL
    };
}

bool PnP::loadCalibration(const std::string& yamlPath) {
    cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;

    fs["cameraMatrix"] >> cameraMatrix_;
    fs["distCoeffs"] >> distCoeffs_;
    fs.release();
    return !cameraMatrix_.empty();
}

bool PnP::estimate(const std::vector<cv::Point2f>& imageCorners,
                              Target3D& target) {
    if (imageCorners.size() != 4 || cameraMatrix_.empty())
        return false;

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(modelPoints_, imageCorners,
                           cameraMatrix_, distCoeffs_,
                           rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

    if (!ok) {
        target.valid = false;
        return false;
    }

    target.x     = tvec.at<double>(0);
    target.y     = tvec.at<double>(1);
    target.z     = tvec.at<double>(2);
    target.rvec  = rvec.clone();
    target.tvec  = tvec.clone();
    target.valid = true;
    return true;
}
