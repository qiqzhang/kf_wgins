#include <iomanip>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <absl/time/clock.h>
#include "common/nav_state.h"
#include "wgins/wgi_engine.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cout << "usage: KF-WGINS dataset/wgins/11.txt" << std::endl;
        return -1;
    }

    std::cout << std::endl << "KF-WGINS: An ESKF-Based Wheel GNSS/INS Integrated Navigation System" << std::endl << std::endl;
    auto ts = absl::Now();

    std::ifstream fin(argv[1]);
    if (!fin) {
        std::cout << "未能找到文件" << std::endl;
        return -1;
    }

    std::ofstream fout("./dataset/wgins/out.txt");
    auto save_vec3 = [](std::ofstream& fout, const Vector3d& v) { 
        std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
        fout << v[0] << " " << v[1] << " " << v[2] << " "; 
    };
    auto save_quat = [](std::ofstream& fout, const Quaterniond& q) {
        std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
        fout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
    };
    auto save_result = [&save_vec3, &save_quat](std::ofstream& fout, const NavStated& save_state) {
        fout << std::setprecision(18) << save_state.timestamp_ << " " << std::setprecision(9);
        save_vec3(fout, save_state.p_);
        save_quat(fout, save_state.R_.unit_quaternion());
        save_vec3(fout, save_state.v_);
        save_vec3(fout, save_state.bg_);
        save_vec3(fout, save_state.ba_);
        fout << std::endl;
    };

    WGIEngine wgiengine;

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (line.empty()) {
            continue;
        }

        if (line[0] == '#') {
            continue;
        }

        // load data from line
        std::stringstream ss;
        ss << line;
        std::string data_type;
        ss >> data_type;

        if (data_type == "IMU") {
            double time, gx, gy, gz, ax, ay, az;
            ss >> time >> gx >> gy >> gz >> ax >> ay >> az;
            auto state = wgiengine.IMUProcessFunc(IMU(time, Vector3d(gx, gy, gz), Vector3d(ax, ay, az)));
            if (state.timestamp_ != 0) {
                save_result(fout, state);
            }
        } else if (data_type == "ODOM") {
            double time, wl, wr;
            ss >> time >> wl >> wr;
            wgiengine.OdomProcessFunc(Odom(time, wl, wr));
        } else if (data_type == "GNSS") {
            double time, lat, lon, alt, heading;
            bool heading_valid;
            ss >> time >> lat >> lon >> alt >> heading >> heading_valid;
            auto state = wgiengine.GNSSProcessFunc(GNSS(time, 4, Vector3d(lat, lon, alt), heading, heading_valid));
            if (state.timestamp_ != 0) {
                save_result(fout, state);
            }
        }
    }
    std::cout << "done." << std::endl;
    return 0;
}
