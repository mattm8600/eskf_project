#include <iostream>
#include <fstream>
#include "eskf.h"
using namespace std;

template<typename Derived>
void printEigen(const Eigen::MatrixBase<Derived>& mat, const std::string& name = "") {
    if (!name.empty()) {
        std::cout << name << ":\n";
    }
    std::cout << mat << std::endl;
}

int main() {
    bool first_run = true;
    fstream FileIn("squareflightdataset.csv");
    fstream FileOut("square_flight_results.csv");
    FileOut << "time,x_hat,y_hat,z_hat,Vx_hat,Vy_hat,Vz_hat,ax_hat,ay_hat,az_hat,ax_bias,ay_bias,az_bias,gx_bias,gy_bias,gz_bias,roll,pitch,yaw,est_heading,mag_correction,pr_correction" << std::endl;
    std::string line;
    getline(FileIn, line);
    eskf kf = eskf();
    float time = 0;
    float t_0 = 0;
    bool use_baro = false;
    bool use_mag = false;
    bool use_gps = false;
    for(int i=0; i<2313;i++) {
        getline(FileIn, line);
        std::stringstream ss(line);  // Use stringstream to extract tokens
        std::string value;
        Eigen::Vector3f acc;
        Eigen::Vector3f gyr;
        Eigen::Vector<float,5> gps;
        Eigen::Vector<float,4> mag;

        // Read in accelerometer
        for(int i=0; i<3;i++) {
            std::getline(ss, value, ',');
            try {
                acc(i) = stof(value);
            }
            catch (const std::invalid_argument& e) {
                std::cerr << "Invalid float value1: " << value << std::endl;
                return 1;
            }
        }
        // Read in Gyro
        for(int i=0; i<3;i++) {
            std::getline(ss, value, ',');
        try {
                gyr(i) = stof(value);
            }
            catch (const std::invalid_argument& e) {
                std::cerr << "Invalid float value2: " << value << std::endl;
                return 1;
            }
        }
        // Try reading in baro
        std::getline(ss,value,',');
        if(value != "-9999999.00") {
            kf.baro_meas = -1*stof(value);
            use_baro = true;
        }
        // Try reading in GPS
        for(int i=0; i< 5; i++) {
            std::getline(ss, value, ',');
            if(value != "-9999999.00") {
            gps[i] = stof(value);
            use_gps = true;
            }
        }
        // Try reading in mag
        for(int i=0; i< 4; i++) {
            std::getline(ss, value, ',');
            if(value != "-9999999.00") {
            mag[i] = stof(value);
            use_mag = true;
            }
        }
        std::getline(ss, value, ',');
        time = stof(value) / (powf(10,6));
        kf.acc_meas = acc;
        kf.gyro_meas = gyr*(1/16.4);
        if(use_gps) {
            kf.gps_x = gps(0);
            kf.gps_y = gps(1);
            kf.gps_z = gps(2);
            kf.gps_hAcc = gps(3);
            kf.gps_vAcc = gps(4);
        }
        if(use_mag) {
            kf.mag_heading = mag(0)*(M_PI/180); // Assumes heading given in degrees, must wrap to +- pi
            kf.mag_heading = kf.wrapToPi(kf.mag_heading);
            kf.mag_vec = Eigen::Vector3f(mag(1),mag(2),mag(3));
        }
        if(first_run) { // ASSUMES LEAVING THE GROUND, For other start point reference GPS change
            t_0 = time;
            kf.nom_state.p = Eigen::Vector3f(0,0,0.05);
            kf.nom_state.vel = Eigen::Vector3f(0,0,0);
            // Eigen::Quaternionf q_init(1, 0, 0, 0);
            // kf.nom_state.q = q_init;
            kf.nom_state.acc_b = Eigen::Vector3f(-0.047293f,-0.01987f,0.002095f);
            kf.nom_state.gyro_b = Eigen::Vector3f(0.006321f,-0.0443f,0.003061f);
            
            // Accelerometer is already in NED
            Eigen::Vector3f acc = (kf.acc_meas - kf.nom_state.acc_b).normalized();
            float pitch = atan2(-acc.x(), std::sqrt(acc.y() * acc.y() + acc.z() * acc.z()));
            float roll  = atan2(acc.y(), acc.z());
            Eigen::Quaternionf q_yaw(Eigen::AngleAxis(kf.mag_heading, Eigen::Vector3f::UnitZ()));
            Eigen::Quaternionf q_roll(Eigen::AngleAxis(roll,  Eigen::Vector3f::UnitX()));
            Eigen::Quaternionf q_pitch(Eigen::AngleAxis(pitch, Eigen::Vector3f::UnitY()));
            Eigen::Quaternionf q_init = q_yaw * q_pitch * q_roll;
            kf.nom_state.q = q_init.normalized();
            printEigen(q_init.toRotationMatrix(), "Initial Rotation");
            first_run = false;
        }
        kf.deltaT = time - t_0;
        // std::cout << kf.deltaT << "\n";
        kf.update(use_baro,use_gps,use_mag);
        t_0 = time;
        cout << "Time: " << time << " ";
        // cout << "Time: " << time << ", Norm of P theta " << kf.P(6,6) << ", " << kf.P(7,7) << ", " << kf.P(8,8) << "\n";
        // printEigen(kf.P, "Covaraince Matrix");
        FileOut << time;
        for (size_t i = 0; i < 3; ++i) {
            FileOut << ",";
            FileOut << kf.nom_state.p[i];
        }
        for (size_t i = 0; i < 3; ++i) {
            FileOut << "," << kf.nom_state.vel[i];
        }
        for (size_t i = 0; i < 3; ++i) {
            FileOut << "," << kf.acc_ned[i];
        }
        for (size_t i = 0; i < 3; ++i) {
            FileOut << "," << kf.nom_state.acc_b[i];
        }
        for (size_t i = 0; i < 3; ++i) {
            FileOut << "," << kf.nom_state.gyro_b[i];
        }
        FileOut << "," << kf.getRoll();
        FileOut << "," << kf.getPitch();
        FileOut << "," << kf.getYaw();
        FileOut << "," << kf.yaw_pred;
        FileOut << "," << kf.mag_used;
        FileOut << "," << kf.pr_used;
        FileOut << "\n";
        use_baro = false;
        use_gps = false;
        use_mag = false;
    }

    FileIn.close();
    FileOut.close();

}