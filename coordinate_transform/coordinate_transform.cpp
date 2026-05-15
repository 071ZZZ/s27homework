#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <map>
#include <vector>

const double PI = std::acos(-1.0);

class Quaternion {
public:
    double w, x, y, z;

    Quaternion(double w = 1.0, double x = 0.0, double y = 0.0, double z = 0.0)
        : w(w), x(x), y(y), z(z) {}

    double norm() const {
        return std::sqrt(w*w + x*x + y*y + z*z);
    }

    Quaternion normalize() const {
        double n = norm();
        if (n == 0) return Quaternion();
        return Quaternion(w/n, x/n, y/n, z/n);
    }

    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }

    std::vector<double> rotate(const std::vector<double>& v) const {
        Quaternion p(0, v[0], v[1], v[2]);
        Quaternion r = (*this) * p * this->conjugate();
        return {r.x, r.y, r.z};
    }

    static Quaternion fromEuler(double yaw, double pitch, double roll) {
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        Quaternion qz(cy, 0, 0, sy);
        Quaternion qy(cp, 0, sp, 0);
        Quaternion qx(cr, sr, 0, 0);
        return qz * qy * qx;   // q = qz * qy * qx
    }

    std::vector<double> toEuler() const {
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (w * y - z * x);
        double pitch;
        if (std::abs(sinp) >= 1.0)
            pitch = std::copysign(PI / 2.0, sinp);
        else
            pitch = std::asin(sinp);

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        return {yaw, pitch, roll};
    }
};

class Pose {
public:
    double x, y, z;
    double yaw, pitch, roll;

    Pose(double x = 0, double y = 0, double z = 0,
         double yaw = 0, double pitch = 0, double roll = 0)
        : x(x), y(y), z(z), yaw(yaw), pitch(pitch), roll(roll) {}

    Quaternion attitudeQuat() const {
        return Quaternion::fromEuler(yaw, pitch, roll);
    }

    void setFromQuat(const Quaternion& q) {
        std::vector<double> euler = q.toEuler();
        yaw   = euler[0];
        pitch = euler[1];
        roll  = euler[2];
    }
};

class Transform {
public:
    Quaternion q; 
    double tx, ty, tz; 

    Transform(double tx = 0, double ty = 0, double tz = 0,
              const Quaternion& q = Quaternion())
        : q(q), tx(tx), ty(ty), tz(tz) {}

    Pose apply(const Pose& src) const {
        std::vector<double> p_src = {src.x, src.y, src.z};
        std::vector<double> p_rot = q.rotate(p_src);
        double newX = p_rot[0] + tx;
        double newY = p_rot[1] + ty;
        double newZ = p_rot[2] + tz;

        Quaternion q_src = src.attitudeQuat();
        Quaternion q_tgt = q * q_src;

        Pose res(newX, newY, newZ, 0, 0, 0);
        res.setFromQuat(q_tgt);
        return res;
    }

    Transform operator*(const Transform& other) const {
        Quaternion q_new = q * other.q;
        std::vector<double> t_rot = q.rotate({other.tx, other.ty, other.tz});
        return Transform(tx + t_rot[0], ty + t_rot[1], tz + t_rot[2], q_new);
    }
};

int main() {
    std::map<std::string, Transform> transforms;
    Transform T_camera_to_gimbal(0.2, 0.0, 0.0);
    Quaternion q_go = Quaternion::fromEuler(-0.1, -0.1, -0.1);
    Transform T_gimbal_to_odom(0.0, 0.0, 0.0, q_go);
    double x, y, z, yaw, pitch, roll;
    std::cin >> x >> y >> z >> yaw >> pitch >> roll;

    std::string target;
    std::string word;
    while (std::cin >> word) {
        if (!word.empty() && word[0] == '/') {
            target = word;
            break;
        }
    }

    Transform T_camera_to_target;

    if (target == "/Gimbal") {
        T_camera_to_target = T_camera_to_gimbal;
    } else if (target == "/Odom") {
        // Camera → Odom = (Gimbal → Odom) * (Camera → Gimbal)
        T_camera_to_target = T_gimbal_to_odom * T_camera_to_gimbal;
    } else {
        std::cerr << "未知目标坐标系: " << target << std::endl;
        return 1;
    }

    Pose srcPose(x, y, z, yaw, pitch, roll);
    Pose tgtPose = T_camera_to_target.apply(srcPose);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << tgtPose.x << " " << tgtPose.y << " " << tgtPose.z << " "
              << tgtPose.yaw << " " << tgtPose.pitch << " " << tgtPose.roll
              << std::endl;

    return 0;
}