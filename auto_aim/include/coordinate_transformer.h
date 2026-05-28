#pragma once

#include "data_types.h"
#include <vector>
#include <cmath>

class Quaternion {
public:
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;

    Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0);

    double norm() const;
    Quaternion normalize() const;
    Quaternion conjugate() const;
    Quaternion operator*(const Quaternion& q) const;
    std::vector<double> rotate(const std::vector<double>& v) const;

    static Quaternion fromEuler(double yaw, double pitch, double roll);
    std::vector<double> toEuler() const;
};

class Pose {
public:
    double x = 0, y = 0, z = 0;
    double yaw = 0, pitch = 0, roll = 0;

    Pose(double x = 0, double y = 0, double z = 0,
         double yaw = 0, double pitch = 0, double roll = 0);

    Quaternion attitudeQuat() const;
    void setFromQuat(const Quaternion& q);
};

class Transform {
public:
    Quaternion q;
    double tx = 0, ty = 0, tz = 0;

    Transform(double tx = 0, double ty = 0, double tz = 0,
              const Quaternion& q = Quaternion());

    Pose apply(const Pose& src) const;
    Transform operator*(const Transform& other) const;
};

class CoordinateTransformer {
public:
    CoordinateTransformer();

    GimbalCommand computeGimbalAngles(const cv::Point3d& cvPos);

    static cv::Point3d cvToRM(const cv::Point3d& cvPt);

private:
    Transform T_camera_to_gimbal_;
};
