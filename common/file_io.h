#pragma once
#include "common/types.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

namespace dr_gins {

std::vector<SensorData> ReadDataFromTxt(const std::string& file_path) {
    std::vector<SensorData> sensor_data_vec;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << file_path << std::endl;
        return sensor_data_vec;
    }

    std::string line;
    while (std::getline(file, line)) {
        // 替换逗号为空格，方便解析
        for (char& c : line) {
            if (c == ',') {
                c = ' ';
            }
        }
        std::stringstream ss(line);
        double timestamp;
        std::string type;
        ss >> timestamp >> type;

        if (type == "rawimus") {
            // format: acc_z, -acc_y, acc_x, gyro_z, -gyro_y, gyro_x
            // IMU frame: Right-Forward-Up
            // Target body frame: Forward-Left-Up
            double acc_z, neg_acc_y, acc_x, gyro_z, neg_gyro_y, gyro_x;
            ss >> acc_z >> neg_acc_y >> acc_x >> gyro_z >> neg_gyro_y >> gyro_x;

            if (!ss.fail()) {
                IMU imu;
                imu.timestamp = timestamp;

                // Coordinate transformation from Right-Forward-Up to Forward-Left-Up
                // Target_X (Forward) = Sensor_Y
                // Target_Y (Left)    = -Sensor_X
                // Target_Z (Up)      = Sensor_Z
                imu.gyro.x() = -neg_gyro_y;
                imu.gyro.y() = -gyro_x;
                imu.gyro.z() = gyro_z;

                imu.accel.x() = -neg_acc_y;
                imu.accel.y() = -acc_x;
                imu.accel.z() = acc_z;

                sensor_data_vec.emplace_back(imu);
            }
        } else if (type == "inspvas") {
            // format: lat, lon, alt, vel_n, vel_e, vel_up, roll, pitch, azimuth
            GNSS gnss;
            gnss.timestamp = timestamp;
            double vel_n, vel_e, vel_up, roll, pitch, azimuth;

            ss >> gnss.lla.x() >> gnss.lla.y() >> gnss.lla.z()
               >> vel_n >> vel_e >> vel_up
               >> roll >> pitch >> azimuth;

            if (!ss.fail()) {
                // NOTE: The current GNSS struct does not store velocity.
                // If you want to use GNSS velocity, add 'Vector3d velocity;' to the GNSS struct in types.h
                // and uncomment the following line:
                gnss.velocity << vel_e, vel_n, vel_up; // ENU velocity

                // Status is not provided in this format, set to a default value
                gnss.status = 0; 
                sensor_data_vec.emplace_back(gnss);
            }
        }
        // Other message types like 'bestutm', 'inscovs', 'IMG' are ignored.
    }

    file.close();
    std::cout << "Successfully read " << sensor_data_vec.size() << " sensor measurements from " << file_path << std::endl;
    return sensor_data_vec;
}
} // namespace dr_gins