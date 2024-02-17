#include <iostream>
#include "StaticIMUInit.h"
#include "common/math_utils.h"

bool StaticIMUInit::AddIMU(const IMU& imu) {
    if (init_success_) {
        return true;
    }

    if (options_.use_speed_for_static_checking_ && !is_static_) {
        std::cout << "等待机器静止" << std::endl;
        init_imu_deque_.clear();
        return false;
    }

    if (init_imu_deque_.empty()) {
        // 记录初始静止时间
        init_start_time_ = imu.time;
    }

    // 记入初始化队列
    init_imu_deque_.push_back(imu);
    // 初始化经过时间
    double init_time = imu.time - init_start_time_;  
    if (init_time > options_.init_time_seconds_) {
        // 尝试初始化逻辑
        TryInit();
    }

    // 维持初始化队列长度
    while (init_imu_deque_.size() > options_.init_imu_queue_max_size_) {
        init_imu_deque_.pop_front();
    }

    current_time_ = imu.time;
    return false;
}

bool StaticIMUInit::AddOdom(const Odom& odom) {
    // 判断车辆是否静止
    if (init_success_) {
        return true;
    }

    if (odom.left_pulse < options_.static_odom_pulse_ && odom.right_pulse < options_.static_odom_pulse_) {
        is_static_ = true;
    } else {
        is_static_ = false;
    }

    current_time_ = odom.time;
    return true;
}

bool StaticIMUInit::TryInit() {
    if (init_imu_deque_.size() < 10) {
        return false;
    }

    // 计算均值和方差
    Vector3d mean_gyro, mean_acce;
    ComputeMeanAndCovDiag(init_imu_deque_, mean_gyro, cov_gyro_, [](const IMU& imu) { return imu.gyro; });
    ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce; });

    // 以 acce 均值为方向，取 9.8 长度为重力
    std::cout << "mean acce: " << mean_acce.transpose() << std::endl;
    gravity_ = -mean_acce / mean_acce.norm() * options_.gravity_norm_;

    // 重新计算加计的协方差
    ComputeMeanAndCovDiag(init_imu_deque_, mean_acce, cov_acce_, [this](const IMU& imu) { return imu.acce + gravity_; });

    // 检查IMU噪声
    if (cov_gyro_.norm() > options_.max_static_gyro_var) {
        std::cout << "陀螺仪测量噪声太大" << cov_gyro_.norm() << " > " << options_.max_static_gyro_var << std::endl;
        return false;
    }

    if (cov_acce_.norm() > options_.max_static_acce_var) {
        std::cout << "加计测量噪声太大" << cov_acce_.norm() << " > " << options_.max_static_acce_var << std::endl;
        return false;
    }

    // 估计测量噪声和零偏
    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    std::cout << "IMU 初始化成功，初始化时间 = " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose() << ", gyro sq = " << cov_gyro_.transpose()
              << ", acce sq = " << cov_acce_.transpose() << ", grav = " << gravity_.transpose()
              << ", norm: " << gravity_.norm() << std::endl;
    std::cout << "mean gyro: " << mean_gyro.transpose() << " acce: " << mean_acce.transpose() << std::endl;
    init_success_ = true;
    return true;
}