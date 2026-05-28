#include "coordinate_transformer.h"
#include <cmath>

Quaternion::Quaternion(double w, double x, double y, double z)
    : w(w), x(x), y(y), z(z) {}

double Quaternion::norm() const {
    return std::sqrt(w*w + x*x + y*y + z*z);
}

Quaternion Quaternion::normalize() const {
    double n = norm();
    if (n < 1e-12) return Quaternion(1, 0, 0, 0);
    return Quaternion(w/n, x/n, y/n, z/n);
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(w, -x, -y, -z);
}

Quaternion Quaternion::operator*(const Quaternion& q) const {
    return Quaternion(
        w*q.w - x*q.x - y*q.y - z*q.z,
        w*q.x + x*q.w + y*q.z - z*q.y,
        w*q.y - x*q.z + y*q.w + z*q.x,
        w*q.z + x*q.y - y*q.x + z*q.w
    );
}

std::vector<double> Quaternion::rotate(const std::vector<double>& v) const {
    Quaternion p(0, v[0], v[1], v[2]);
    Quaternion result = (*this) * p * this->conjugate();
    return {result.x, result.y, result.z};
}

Quaternion Quaternion::fromEuler(double yaw, double pitch, double roll) {
    double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);

    double w = cr*cp*cy + sr*sp*sy;
    double x = sr*cp*cy - cr*sp*sy;
    double y = cr*sp*cy + sr*cp*sy;
    double z = cr*cp*sy - sr*sp*cy;

    return Quaternion(w, x, y, z);
}

std::vector<double> Quaternion::toEuler() const {
    double sinp = 2.0 * (w*y - z*x);
    double pitch;
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double roll  = std::atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y));
    double yaw   = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));

    return {yaw, pitch, roll};  // 返回顺序: yaw, pitch, roll
}

Pose::Pose(double x, double y, double z, double yaw, double pitch, double roll)
    : x(x), y(y), z(z), yaw(yaw), pitch(pitch), roll(roll) {}

Quaternion Pose::attitudeQuat() const {
    return Quaternion::fromEuler(yaw, pitch, roll);
}

void Pose::setFromQuat(const Quaternion& q) {
    auto euler = q.toEuler();
    yaw   = euler[0];
    pitch = euler[1];
    roll  = euler[2];
}

Transform::Transform(double tx, double ty, double tz, const Quaternion& q)
    : q(q), tx(tx), ty(ty), tz(tz) {}

Pose Transform::apply(const Pose& src) const {
    auto p_rot = q.rotate({src.x, src.y, src.z});
    Quaternion q_new = q * src.attitudeQuat();
    Pose result(p_rot[0] + tx, p_rot[1] + ty, p_rot[2] + tz);
    result.setFromQuat(q_new);
    return result;
}

Transform Transform::operator*(const Transform& other) const {
    Quaternion q_new = this->q * other.q;
    auto p = this->q.rotate({other.tx, other.ty, other.tz});
    return Transform(this->tx + p[0], this->ty + p[1], this->tz + p[2], q_new);
}

CoordinateTransformer::CoordinateTransformer() {
    T_camera_to_gimbal_ = Transform(0.2, 0.0, 0.0, Quaternion());
}

cv::Point3d CoordinateTransformer::cvToRM(const cv::Point3d& cvPt) {
    // OpenCV camera: X右 Y下 Z前  →  RM: X前 Y左 Z上
    return cv::Point3d(cvPt.z, -cvPt.x, -cvPt.y);
}

GimbalCommand CoordinateTransformer::computeGimbalAngles(const cv::Point3d& cvPos) {
    GimbalCommand cmd;

    cv::Point3d rmPos = cvToRM(cvPos);

    // 在坐标系中应用相机→云台的偏移
    Pose targetInCamera(rmPos.x, rmPos.y, rmPos.z, 0, 0, 0);
    Pose targetInGimbal = T_camera_to_gimbal_.apply(targetInCamera);

    double gx = targetInGimbal.x;
    double gy = targetInGimbal.y;
    double gz = targetInGimbal.z;

    double horizontalDist = std::sqrt(gx*gx + gy*gy);
    cmd.yaw   = std::atan2(gy, gx);
    cmd.pitch = std::atan2(gz, horizontalDist);
    cmd.roll  = 0.0;
    cmd.target_locked = true;

    return cmd;
}
