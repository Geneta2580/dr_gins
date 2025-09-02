#pragma once

#include <string>
#include <variant>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>

namespace dr_gins {

// Eigen 类型别名，简化使用
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Vector4d = Eigen::Vector4d;
using Matrix4d = Eigen::Matrix4d;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Vector6d = Eigen::Matrix<double, 6, 1>; 
using Quaterniond = Eigen::Quaterniond;
template<typename Scalar, int Rows, int Cols>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;

// IMU数据结构
struct IMU {
    double timestamp = 0.0;
    Vector3d gyro = Vector3d::Zero();
    Vector3d accel = Vector3d::Zero();
    double dt = 0.0;
};

// GNSS数据结构
struct GNSS {
    double timestamp = 0.0;
    Vector3d lla = Vector3d::Zero(); // 纬度(lat), 经度(lon), 高程(alt)
    Vector3d velocity = Vector3d::Zero(); // 速度
    double horizontal_accuracy = 0.0;
    int status = 0; // 状态
};

struct NavState {
    double timestamp = 0.0;                                     // 时间戳
    Quaterniond q = Quaterniond::Identity(); // 姿态 (使用四元数表示) qnb
    Vector3d p = Vector3d::Zero();         // 位置 (通常在某个局部坐标系下)
    Vector3d v = Vector3d::Zero();         // 速度
    Vector3d bg = Vector3d::Zero();          // 陀螺仪偏置估计
    Vector3d ba = Vector3d::Zero();         // 加速度计偏置估计
    Vector3d g = Vector3d(0, 0, -9.81);      // 重力加速度，可估计
};

struct TruePose {
    double timestamp = 0.0;
    Quaterniond q = Quaterniond::Identity(); // 姿态 (使用四元数表示)
    Vector3d p = Vector3d::Zero();         // 位置 (通常在某个局部坐标系下)
};

// 历史状态
struct HistoryState {
    double timestamp;
    NavState state;
    Matrix<double, 18, 18> P = Eigen::Matrix<double, 18, 18>::Zero();
    IMU imu_data;
};

// 使用std::variant来封装不同类型的传感器数据
using SensorData = std::variant<IMU, GNSS, TruePose>;

std::vector<SensorData> ReadDataFromTxt(const std::string& file_path);

} // namespace dr_gins