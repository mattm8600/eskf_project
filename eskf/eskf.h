#ifdef ARDUINO
#include <Arduino.h>
#define DMAMEM_ATTR DMAMEM
#else
#include <iostream>  // for desktop printing
#define DMAMEM
#endif

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
#include <iostream>

class eskf{
    public:
        // All coordinates are expected in NED, rad, m
        struct State {
            Eigen::Vector3f p;// Position Vector
            Eigen::Vector3f vel; // Velocity Vector
            Eigen::Quaternionf q; // Nominal Quaternion 
            Eigen::Vector3f acc_b; // Accelerometer Bias Vector
            Eigen::Vector3f gyro_b; // Gyro Bias Vector
            State()
            : p(Eigen::Vector3f::Zero()),
            vel(Eigen::Vector3f::Zero()),
            q(Eigen::Quaternionf::Identity()),
            acc_b(Eigen::Vector3f::Zero()),
            gyro_b(Eigen::Vector3f::Zero()) {}
        };


        // Nominal State (the estimated state of the drone)
        DMAMEM static State nom_state;

        // MEASUREMENTS:
            // IMU
            DMAMEM static Eigen::Vector3f acc_meas;
            DMAMEM static Eigen::Vector3f gyro_meas;
            Eigen::Vector3f omega_filtered = Eigen::Vector3f::Zero();
            float gyr_alpha = 0.9f;
            // GPS
            DMAMEM static Eigen::Vector<float,6> gps_meas;
            float gps_heading;
            // BARO
            float baro_meas;
            // MAG
            float mag_heading;
            DMAMEM static Eigen::Vector3f mag_vec;
            Eigen::Vector3f filtered_mag_vec = Eigen::Vector3f::Zero();
            float mag_vec_alpha = 0.9f;
        
        // NOISE:
            // IMU Process Noise
            float sigma_a_n = 0.0018; // m/s^2 / sqrt(Hz)
            float sigma_w_n = 0.01f;
            // IMU Bias Walk
            float sigma_a_w = 1e-5f; // m/s^2 / sqrt(s)
            float sigma_w_w = 1e-3f;

            // GPS
            float gps_sAcc;
            float gps_cAcc;
            float gps_hAcc;
            float gps_vAcc;
            // BARO
            float baro_noise = 0.075f;
            float baro_dev_tol = 3.0f;
            // MAG
            float mag_noise = 0.05f;
            float predicted_grav_noise = 0.03f;

        // Covariance
        DMAMEM static Eigen::Matrix<float,15,15> P;
        float pr_cov_infl = 1.1f;
        float yaw_cov_infl = 1.1f;

        // DEBUG
        float y_yaw = 0.0;
        float yaw_pred = 0.0;
        bool mag_used = false;
        bool pr_used = false;
        bool gps_yaw_used = false;

        // TAKEOFF REQUIREMENTS
        bool gps_lock_acquired = false;
        bool gyr_bias_est =false;
        DMAMEM static Eigen::Vector3f acc_ned;
        float deltaT;

        eskf();
        bool update(bool new_baro, bool new_gps, bool new_mag, uint32_t timestamp);
        bool set_initial_state(Eigen::Vector3f position = Eigen::Vector3f::Zero(), Eigen::Vector3f velocity = Eigen::Vector3f::Zero(), Eigen::Vector3f acc_b = Eigen::Vector3f(-0.047293f,-0.01987f,0.002095f));
        float get_pitch();
        float get_roll();
        float get_yaw();
        float wrap_to_pi(float angle);
        void predict_covariance(Eigen::Matrix3f R, Eigen::Matrix3f acc_skew);
        void predict_state();
        bool fuse_sensors(bool new_mag, bool new_baro, bool new_gps);
        
    private:
        // Internal Functions
        
        Eigen::Matrix3f skewSymmetric(const Eigen::Vector3f& v);
        DMAMEM static Eigen::Vector3f g;
        
        // Gyro Bias Estimation
        uint32_t t_km1 = 0;
        int gyr_sample_count = 0;
        DMAMEM static Eigen::Matrix<float,15,15> Fx;
        
};
#endif