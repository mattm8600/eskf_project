#undef B0
#undef B1
#undef B2
#undef B3
#undef B4
#undef B5
#undef B6
#undef B7
#undef F

#ifndef eskf_h
#define eskf_h

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Cholesky" 
#include "Eigen/LU" 
#include <iostream>

class eskf{
    public:
        float deltaT;
        // Everything is in the North East Down Frame
        struct State {
            Eigen::Vector3f p;// Position Vector
            Eigen::Vector3f vel; // Velocity Vector
            Eigen::Quaternionf q; // Nominal Quaternion 
            Eigen::Vector3f acc_b; // Accelerometer Bias Vector
            Eigen::Vector3f gyro_b; // Gyro Bias Vector
        };

        // Are there new GPS, Baro, Magnetometer Readings?
        bool new_mag;
        bool new_gps;


        // IMU Measurements
        Eigen::Vector3f acc_meas;
        Eigen::Vector3f gyro_meas;
        
        // GPS, Baro, magnetometer Measurements
        float gps_x;
        float gps_y;
        float gps_z;


        // DENIBUG
        Eigen::Matrix<float,15,1> error_pred;
        Eigen::Vector3f acc_ned;
        // TODO: Incorporate GPS yaw control
        float baro_meas;
        float mag_heading;
        Eigen::Vector3f mag_vec;
        
        // States
        State nom_state;

        // White noise densities (from BMI270 datasheet)
        float sigma_a_n = 0.001569; // m/s^2 / sqrt(Hz)
        float sigma_w_n = 0.0001222; // rad/s / sqrt(Hz)

        // Random walk (Estimated from ChatGPT)
        float sigma_a_w = 5e-4; // m/s^2 / sqrt(s)
        float sigma_w_w = 5e-5; // rad/s / sqrt(s)

        // Sensor Noise

        // GPS Noise is already in one std dev units, taken directly from the GPS
        float gps_hAcc;
        float gps_vAcc;
        // From baro datasheet
        float baro_noise = 0.005f; // Approximate error of +- .005m
        float baro_dev_tol = 3.0f;
        float mag_noise = 0.5; // Taken from .32 microT and .41microT noise from sheet
        Eigen::Matrix<float,15,15> P;
        eskf();
        bool update(bool new_baro, bool new_gps, bool new_mag);
    private:
        void predict_covariance(Eigen::Matrix3f R, Eigen::Matrix3f acc_skew);
        void predict_state();
        eskf::State fuse_sensors(bool new_mag, bool new_baro, bool new_gps);
        Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v);
        Eigen::Vector3f g;
        
};

#endif