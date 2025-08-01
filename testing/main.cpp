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
    fstream FileIn("tri_flight_3.csv");
    fstream FileOut("tri_flight_3_results.csv");
    FileOut << "time,x_hat,y_hat,z_hat,Vx_hat,Vy_hat,Vz_hat,ax_hat,ay_hat,az_hat,ax_bias,ay_bias,az_bias,gx_bias,gy_bias,gz_bias,roll,pitch,yaw,est_heading,mag_correction,pr_correction,P_trace,gps_yaw_correction" << std::endl;
    std::string line;
    getline(FileIn, line);
    eskf kf = eskf();
    float time = 0;
    float t_0 = 0;
    bool use_baro = false;
    bool use_mag = false;
    bool use_gps = false;
    bool gps_lock = false;
    for(int i=0; i<4565;i++) {
        getline(FileIn, line);
        std::stringstream ss(line);
        std::string value;
        Eigen::Vector3f acc;
        Eigen::Vector3f gyr;
        Eigen::Vector<float,11> gps;
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
            kf.baro_meas = stof(value);
            use_baro = true;
        }
        // Try reading in GPS
        for(int i=0; i<11; i++) {
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
        kf.gyro_meas = gyr;
        if(use_gps) {
            kf.gps_meas = Eigen::Vector<float,6>(gps(0),gps(1),gps(2)-0.75f,gps(5),gps(6),gps(7));
            kf.gps_heading = gps(9);
            kf.gps_hAcc = gps(3);
            kf.gps_vAcc = gps(4);
            kf.gps_sAcc = gps(8);
            kf.gps_cAcc = gps(10);
        }
        if(use_mag) {
            kf.mag_heading = mag(0);
            kf.mag_heading = kf.wrap_to_pi(kf.mag_heading);
            kf.mag_vec = Eigen::Vector3f(mag(1),mag(2),mag(3));
        }
        if(first_run) { // ASSUMES LEAVING THE GROUND, For other start point reference GPS change
            kf.set_initial_state();
            first_run = false;
        }
        if(kf.update(use_baro,use_gps,use_mag,time)) {
            {
                kf.predict_state();
            }
            {
                kf.fuse_sensors(use_baro,use_gps, use_mag);
            }
        }
        std::cout << "Time: " << time << "\n";
        
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
        FileOut << "," << kf.get_pitch();
        FileOut << "," << kf.get_roll();
        FileOut << "," << kf.get_yaw();
        FileOut << "," << kf.yaw_pred;
        FileOut << "," << kf.mag_used;
        FileOut << "," << kf.pr_used;
        FileOut << "," << kf.P.trace();
        FileOut << "," << kf.gps_yaw_used;
        FileOut << "\n";
        use_baro = false;
        use_gps = false;
        use_mag = false;
    }

    FileIn.close();
    FileOut.close();

}