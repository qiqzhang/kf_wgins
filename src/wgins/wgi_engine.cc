#include <unistd.h>
#include "utm_convert.h"
#include "wgi_engine.h"

WGIEngine::WGIEngine() {
    eskf     = std::make_shared<ESKFD>();
    imu_init = std::make_shared<StaticIMUInit>();
}

// IMU 处理函数
NavStated WGIEngine::IMUProcessFunc(const IMU &imu) {
    NavStated state;

    if (!imu_init->InitSuccess()) {
        imu_init->AddIMU(imu);
        return state;
    }
    // 需要IMU初始化
    if (!imu_inited) {
        // 读取初始零偏，设置ESKF
        ESKFD::Options options;
        // 噪声由初始化器估计
        options.gyro_var_ = sqrt(imu_init->GetCovGyro()[0]);
        options.acce_var_ = sqrt(imu_init->GetCovAcce()[0]);
        eskf->SetInitialConditions(options, imu_init->GetInitBg(), imu_init->GetInitBa(), imu_init->GetGravity());
        imu_inited = true;
        return state;
    }
    if (!gnss_inited) {
        // 等待有效的RTK数据
        return state;
    }

    // GNSS 也接收到之后，再开始进行预测
    eskf->Predict(imu);

    // predict就会更新ESKF，所以此时就可以发送数据
    state = eskf->GetNominalState();
    usleep(1e3);
    return state;
}


NavStated WGIEngine::GNSSProcessFunc(const GNSS &gnss) {
    NavStated state;

    // GNSS 处理函数
    if (!imu_inited) {
        return state;
    }

    GNSS gnss_convert = gnss;
    Eigen::Vector2d antenna_pos(gnss_convert.antenna_pox_x, gnss_convert.antenna_pox_y);
    if (!ConvertGps2UTM(gnss_convert, antenna_pos, gnss_convert.antenna_angle) || !gnss_convert.heading_valid_) {
        return state;
    }

    // 去掉原点
    if (!first_gnss_set) {
        origin = gnss_convert.utm_pose_.translation();
        first_gnss_set = true;
    }
    gnss_convert.utm_pose_.translation() -= origin;

    // 要求RTK heading有效，才能合入ESKF
    eskf->ObserveGps(gnss_convert);

    state = eskf->GetNominalState();
    gnss_inited = true;
    return state;
}

void WGIEngine::OdomProcessFunc(const Odom& odom) {
    imu_init->AddOdom(odom);
    if (imu_inited && gnss_inited) {
        eskf->ObserveWheelSpeed(odom);
    }
}

