/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef TYPES_H
#define TYPES_H

#include <memory>
#include <Eigen/Geometry>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector2d;

/// GNSS状态位信息
/// 通常由GNSS厂商提供，这里使用千寻提供的状态位
enum class GpsStatusType {
    GNSS_FLOAT_SOLUTION = 5,         // 浮点解（cm到dm之间）
    GNSS_FIXED_SOLUTION = 4,         // 固定解（cm级）
    GNSS_PSEUDO_SOLUTION = 2,        // 伪距差分解（分米级）
    GNSS_SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
    GNSS_NOT_EXIST = 0,              // GPS无信号
    GNSS_OTHER = -1,                 // 其他
};

/// UTM 坐标
struct UTMCoordinate {
    UTMCoordinate() = default;
    explicit UTMCoordinate(int zone, const Vector2d& xy = Vector2d::Zero(), bool north = true)
        : zone_(zone), xy_(xy), north_(north) {}

    int zone_ = 0;              // utm 区域
    Vector2d xy_ = Vector2d::Zero();  // utm xy
    double z_ = 0;              // z 高度（直接来自于gps）
    bool north_ = true;         // 是否在北半球
};

typedef struct GNSS {
    GNSS() = default;
    GNSS(double unix_time, int status, const Vector3d& lat_lon_alt, double heading, bool heading_valid)
        : time(unix_time), lat_lon_alt_(lat_lon_alt), heading_(heading), heading_valid_(heading_valid) {
        status_ = GpsStatusType(status);
    }
    GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST;  // GNSS 状态位
    Vector3d lat_lon_alt_ = Vector3d::Zero();               // 经度、纬度、高度，前二者单位为度
    double heading_ = 0.0;                                  // 双天线读到的方位角，单位为度
    bool heading_valid_ = false;                            // 方位角是否有效

    UTMCoordinate utm_;       // UTM 坐标（区域之类的也在内）
    bool utm_valid_ = false;  // UTM 坐标是否已经计算（若经纬度给出错误数值，此处也为false）
    Sophus::SE3d utm_pose_;   // 用于后处理的6DoF Pose

    // RTK天线安装偏角
    float antenna_angle = 12.06;
    // RTK天线安装偏移X
    float antenna_pox_x = -0.17;
    // RTK天线安装偏移Y
    float antenna_pox_y = -0.20;
    double time = 0;                                         // unix系统时间
    // 经纬度高程坐标
    Vector3d blh;
    Vector3d std;

    bool isvalid;
} GNSS;

typedef struct IMU {
    IMU() = default;
    IMU(double t, const Vector3d& gyr, const Vector3d& acc) : time(t), gyro(gyr), acce(acc) {}
    Vector3d gyro = Vector3d::Zero();
    Vector3d acce = Vector3d::Zero();

    double time = 0.0;
    double dt;

    Vector3d dtheta;
    Vector3d dvel;

    double odovel;
} IMU;

struct Odom {
    Odom() {}
    Odom(double t, double left_pulse_, double right_pulse_): time(t), left_pulse(left_pulse_), right_pulse(right_pulse_) {}

    double time = 0.0;

    // 左右轮的单位时间转过的脉冲数
    double left_pulse = 0.0;  
    double right_pulse = 0.0;
};

typedef struct Pose {
    Matrix3d R;
    Vector3d t;
} Pose;

using GNSSPtr = std::shared_ptr<GNSS>;

#endif // TYPES_H
