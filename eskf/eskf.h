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
        bool takeoff = false;


        // IMU Measurements
        bool gps_lock_acquired = false;
        Eigen::Vector3f acc_meas;
        Eigen::Vector3f gyro_meas;
        Eigen::Vector3f omega_filtered = Eigen::Vector3f::Zero();
        float gyr_alpha = 0.9f;
        float pr_cov_infl = 1.1f;
        float yaw_cov_infl = 1.1f;
        
        // GPS, Baro, magnetometer Measurements
        Eigen::Vector<float,6> gps_meas;
        float gps_heading;
        // DEBUG
        Eigen::Matrix<float,15,1> error_pred;
        Eigen::Vector3f acc_ned;



        // TODO: Incorporate GPS yaw control
        float baro_meas;
        float mag_heading;
        Eigen::Vector3f mag_vec;
        Eigen::Vector3f filtered_mag_vec = Eigen::Vector3f::Zero();
        float mag_vec_alpha = 0.9f;
        
        // States
        State nom_state;

        // BEFORE TAKEOFF NOISE
        // // White noise densities (from BMI270 datasheet)
        float sigma_a_n = 0.0018; // m/s^2 / sqrt(Hz)
        // float sigma_w_n = 0.0001222; // rad/s / sqrt(Hz)
        float sigma_w_n = 0.01f;
        // Random walk (Estimated from ChatGPT)
        float sigma_a_w = 1e-5f; // m/s^2 / sqrt(s)
        float sigma_w_w = 1e-3f;

        // AFTER TAKEOFF NOISE
        // // White noise densities (from BMI270 datasheet)
        // float sigma_a_n = 0.05f; // m/s^2 / sqrt(Hz)
        // // float sigma_w_n = 0.0001222; // rad/s / sqrt(Hz)
        // float sigma_w_n = 0.04f;
        // // Random walk (Estimated from ChatGPT)
        // float sigma_a_w = 5e-3f; // m/s^2 / sqrt(s)
        // float sigma_w_w = 1e-3f;



        // GPS Noise is already in one std dev units, taken directly from the GPS
        float gps_sAcc;
        float gps_cAcc;
        float gps_hAcc;
        float gps_vAcc;
        // From baro datasheet
        float baro_noise = 0.1f; // Approximate error of +- .005m
        float baro_dev_tol = 3.0f;
        float mag_noise = 0.05f*0.05f; // Taken from .32 microT and .41microT noise from sheet
        float predicted_grav_noise = 0.05f;

        // DEBUG
        float y_yaw = 0.0;
        float yaw_pred = 0.0;
        bool mag_used = false;
        bool pr_used = false;
        bool gps_yaw_used = false;

        eskf();
        bool update(bool new_baro, bool new_gps, bool new_mag);
        Eigen::Matrix<float,15,15> P;
        float getPitch();
        float getRoll();
        float getYaw();
        float wrapToPi(float angle);
    private:
        void predict_covariance(Eigen::Matrix3f R, Eigen::Matrix3f acc_skew);
        void predict_state();
        eskf::State fuse_sensors(bool new_mag, bool new_baro, bool new_gps);
        Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v);
        Eigen::Vector3f g;
        Eigen::Vector3f B_ned;
};
#endif