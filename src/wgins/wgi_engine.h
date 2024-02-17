#pragma once

#include <deque>
#include <memory>
#include <Eigen/Core>
#include "common/nav_state.h"
#include "eskf.h"
#include "StaticIMUInit.h"

class WGIEngine {

public:
    WGIEngine();

    NavStated IMUProcessFunc(const IMU &imu);
    NavStated GNSSProcessFunc(const GNSS &gnss);
    void OdomProcessFunc(const Odom& odom);

private:
    bool imu_inited = false;
    bool gnss_inited = false;
    bool first_gnss_set = false;
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();

    std::shared_ptr<ESKFD> eskf;
    std::shared_ptr<StaticIMUInit> imu_init;     
};