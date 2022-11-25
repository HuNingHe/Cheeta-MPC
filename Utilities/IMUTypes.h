//
// Created by hun on 22-5-12.
//

#ifndef MAIN_CPP_IMU_TYPES_H
#define MAIN_CPP_IMU_TYPES_H
#include "MathTypes.h"

struct VectorNavData {
    VectorNavData(){
        for (int i = 0; i < 3; ++i) {
            accelerometer[i] = 0;
            gyro[i] = 0;
        }
        quat.w() = 1;
        quat.x() = 0;
        quat.y() = 0;
        quat.z() = 0;
    }
    Vec3<double> accelerometer;
    Vec3<double> gyro;
    Quat<double> quat; // w x y z
};

struct SensorNoise {
    explicit SensorNoise(double dt){
        controller_dt = dt;
        imu_process_noise_position = 0.02;
        imu_process_noise_velocity = 0.02;
        foot_process_noise_position = 0.002;
        foot_sensor_noise_position = 0.001;
        foot_sensor_noise_velocity = 0.05;
        foot_height_sensor_noise = 0.001;
    }
    double imu_process_noise_position;
    double imu_process_noise_velocity;
    double foot_process_noise_position;
    double foot_sensor_noise_position;
    double foot_sensor_noise_velocity;
    double foot_height_sensor_noise;
    double controller_dt;
};
#endif //MAIN_CPP_IMU_TYPES_H
