#include "eskf.h"
 #include "Eigen/Cholesky"

// From IMU Datasheet:
/*
    accelerometer noise, gyro noise
*/
DMAMEM eskf::State eskf::nom_state;;
DMAMEM Eigen::Vector3f eskf::g = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
DMAMEM Eigen::Matrix<float,15,15> eskf::P = Eigen::Matrix<float,15,15>::Identity();
DMAMEM Eigen::Matrix<float,15,15> eskf::Fx = Eigen::Matrix<float,15,15>::Zero();
DMAMEM Eigen::Vector3f eskf::acc_meas = Eigen::Vector3f::Zero();
DMAMEM Eigen::Vector3f eskf::gyro_meas = Eigen::Vector3f::Zero();
DMAMEM Eigen::Vector<float,6> eskf::gps_meas = Eigen::Matrix<float,6,1>::Zero();
DMAMEM Eigen::Vector3f eskf::mag_vec = Eigen::Vector3f::Zero();
DMAMEM Eigen::Vector3f eskf::acc_ned = Eigen::Vector3f::Zero();


// Perturbation impulses vector
eskf::eskf() {
    // Set start state
    P.block<3,3>(6,6) = 0.01 * Eigen::Matrix3f::Identity(); // Attitude
    P.block<3,3>(9,9)   *= 1e-4;  // accel bias
    P.block<3,3>(12,12) *= 1e-4;  // gyro bias
    std::cout << P.trace() << "\n";

}


/*  Name: Update Function
    
    Description: This function performs one timestep update of the filter as well as handles startup checks
    
    Parameters:
    - new_baro -> Whether a new barometer measurement has been received
    - new_gps -> Whether new GPS position and GPS velocity measurements have been received
    - new_mag -> Whether a new magnetometer measurement has been received
    - timestamp -> The timestamp of the recieved data  

    Output: 
    - True -> The filter ran successfully and nom_state has updated
    - False -> The filter has not ran successfully for one of the following reasons:
        Accurate GPS has not been received yet
        Gyro Bias has not been estimated yet (First 200 Samples without motion)
        The Filter its-self failed (Should throw an error if I actually implemented that lol)
*/
bool eskf::update(bool new_baro, bool new_gps, bool new_mag, uint32_t timestamp) {
    bool success = false;
    // Initialize first time step
    if(t_km1 == 0) {
        t_km1 = timestamp;
    }
    // Calculate time elapsed for IMU propogation
    deltaT = (timestamp - t_km1)*powf(10,-6);
    t_km1 = timestamp;
    if(gps_lock_acquired && gyr_bias_est) {
        return true;
    }

    else if(!gyr_bias_est && gyro_meas.norm() < 0.02f) {
        // Estimate the gyro bias by taking the average of stationary samples
        nom_state.gyro_b += gyro_meas;
        gyr_sample_count ++;
        if(gyr_sample_count > 200) {
            nom_state.gyro_b = nom_state.gyro_b / gyr_sample_count;
            gyr_bias_est = true;
            std::cout << "Gyro Estimated" << "\n";
        }
    }

    // Ensure that we have GPS position within our threshold accuracy
    else if(!gps_lock_acquired && new_gps && gps_hAcc > 0 && gps_vAcc > 0 && gps_hAcc < 5 && gps_vAcc < 6) {
        Eigen::Vector3f acc = acc_meas - nom_state.acc_b;
        acc = acc.normalized();
        // Initialize the quaternion start state
        
        float pitch = atan2(-acc.x(), std::sqrt(acc.y() * acc.y() + acc.z() * acc.z()));
        Eigen::AngleAxisf yaw_aa = Eigen::AngleAxisf(mag_heading, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf  pitch_aa = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());
        Eigen::Quaternionf q_yaw = Eigen::Quaternionf(yaw_aa);
        Eigen::Quaternionf q_pitch = Eigen::Quaternionf(pitch_aa);
        Eigen::Quaternionf q_tmp = q_yaw * q_pitch;
        Eigen::Quaternionf q_roll;
        float roll  = atan2(acc.y(), acc.z());
        Eigen::AngleAxisf roll_aa = Eigen::AngleAxisf(roll,  Eigen::Vector3f::UnitX());
        q_roll = Eigen::Quaternionf(roll_aa);
        Eigen::Quaternionf q_init = q_tmp * q_roll;
        q_init = q_init.normalized();
        nom_state.q = q_init;
        gps_lock_acquired = true;
        std::cout << "Init quaternions" << "\n";
    }

    return success; 
}
        
/*  Name: Initial State Setter Function
    
    Description: Set the initial Position, Velocity, and Acceleration Bias.
    
    Parameters:
    - pos -> Eigen Vector with the starting NED position
    - vel -> Eigen Vector with the starting NED velocity
    - acc_b -> Put your experimentally determined accelerometer bias here. Preloaded with my initial bias.
*/
bool eskf::set_initial_state(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f acc_b) {
    nom_state.p = pos;
    nom_state.vel = vel;
    nom_state.acc_b = acc_b;
    nom_state.gyro_b = Eigen::Vector3f::Zero();
    return true;
}





/*  Name: Initial State Setter Function
    
    Description: Performs one timestep of IMU propogation

    Output: Updates the nominal state with the most recent measurements */
void eskf::predict_state() {

    // Get Rot. matrix
    nom_state.q.normalize();
    Eigen::Matrix3f R = nom_state.q.toRotationMatrix();

    acc_ned = R *(acc_meas - nom_state.acc_b)-g; 
    // Propogate the new acceleration into position, velocity
    nom_state.p = nom_state.p + nom_state.vel*deltaT + R*acc_ned*0.5f*deltaT*deltaT;
    nom_state.vel = nom_state.vel + acc_ned*deltaT;
    nom_state.acc_b = nom_state.acc_b;
    nom_state.gyro_b = nom_state.gyro_b;

    Eigen::Vector3f omega = gyro_meas - nom_state.gyro_b;
    // Apply a first order low pass filter to smooth the angular velocity
    omega_filtered = gyr_alpha*omega + (1-gyr_alpha)*omega_filtered;
    // Propogate the angular velocity into new orientation
    float angle = omega_filtered.norm() * deltaT;
    Eigen::Vector3f axis = (angle < 1e-5f) ? omega_filtered : omega_filtered.normalized();
    Eigen::Quaternionf dq(Eigen::AngleAxisf(angle, axis));
    nom_state.q = (nom_state.q * dq).normalized();
    
    // Required for Covariance Update
    Eigen::Vector3f acc_diff = acc_meas - nom_state.acc_b;
    Eigen::Matrix3f acc_skew = skewSymmetric(acc_diff);
    predict_covariance(R,acc_skew);
}

/*  Name: Covariance Propogation Function
    
    Description: 
        Updates the covariance matrix P for the changes in position, orientation.


    Output: Updates the nominal state with the most recent measurements */
void eskf::predict_covariance(Eigen::Matrix3f R, Eigen::Matrix3f acc_skew) {

    // Calculate error State Jacobian (F_x)
    Fx.block<3,3>(0, 0) = Eigen::Matrix3f::Identity(); 
    Fx.block<3,3>(3, 3) = Eigen::Matrix3f::Identity(); 
    Fx.block<3,3>(9, 9) = Eigen::Matrix3f::Identity(); 
    Fx.block<3,3>(12, 12) = Eigen::Matrix3f::Identity();  
    Fx.block<3,3>(0, 3) = Eigen::Matrix3f::Identity()*deltaT;
    Fx.block<3,3>(6, 6) = R.transpose();
    Fx.block<3,3>(3, 9) = -1*R*deltaT;
    Fx.block<3,3>(6, 12) = -Eigen::Matrix3f::Identity()*deltaT;
    Eigen::Matrix3f temp = (-1*R) * acc_skew;
    Fx.block<3,3>(3, 6) = temp * deltaT;

    // Calculate Jacobian wrt perturbation
    Eigen::Matrix<float, 15, 12> Fi;
    Fi.setZero();
    Fi.block<3,3>(3, 0) = Eigen::Matrix3f::Identity(); 
    Fi.block<3,3>(6, 3) = Eigen::Matrix3f::Identity(); 
    Fi.block<3,3>(9, 6) = Eigen::Matrix3f::Identity(); 
    Fi.block<3,3>(12, 9) = Eigen::Matrix3f::Identity(); 
    
    Eigen::Matrix<float,12,12> Qi;
    Qi.setZero();
    Qi.block<3,3>(0, 0) = powf(sigma_a_n,2.0f)*deltaT*Eigen::Matrix3f::Identity();
    Qi.block<3,3>(3, 3) = powf(sigma_w_n,2.0f)*deltaT*Eigen::Matrix3f::Identity();
    Qi.block<3,3>(6, 6) = powf(sigma_a_w,2.0f)*deltaT*Eigen::Matrix3f::Identity(); 
    Qi.block<3,3>(9, 9) = powf(sigma_w_w,2.0f)*deltaT*Eigen::Matrix3f::Identity();  
    // Update Covariance matrix (P)
    P = Fx*P*Fx.transpose() + Fi*Qi*Fi.transpose();
    
}

bool eskf::fuse_sensors(bool new_mag, bool new_baro, bool new_gps) {

    Eigen::Vector3f d_theta_mag = Eigen::Vector3f::Zero();
    Eigen::Vector3f d_theta_grav = Eigen::Vector3f::Zero();
    Eigen::Vector3f d_theta_gps = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_grav = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_mag = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_gps = Eigen::Vector3f::Zero();

    // Use the measured gravity to correct pitch and roll:
    float acc_norm = (acc_meas - nom_state.acc_b).norm();
    float speed_norm = nom_state.vel.norm();
    float angle = acosf((acc_meas-nom_state.acc_b).normalized().dot(nom_state.q.toRotationMatrix().transpose()*g.normalized()));
    if (fabsf(angle) < .175 || (acc_norm >= 9.0f && acc_norm <= 10.5f) ) {
        pr_used = true;
        nom_state.q.normalize();
        Eigen::Matrix3f R = nom_state.q.toRotationMatrix();
        Eigen::Vector3f grav_pred = R.transpose()*g;
        Eigen::Matrix<float,3,15> H_grav = Eigen::Matrix<float,3,15>::Zero();
        H_grav.block<3,3>(0,6) = -1*skewSymmetric(grav_pred);
        Eigen::Vector3f grav_meas = (acc_meas - nom_state.acc_b).normalized() * 9.81f;
        Eigen::Matrix3f R_grav = predicted_grav_noise * Eigen::Matrix3f::Identity();

        Eigen::Vector3f y_grav = grav_pred-grav_meas;
        Eigen::Matrix<float,3,15> HPHt = H_grav * P;
        Eigen::Matrix3f S_intermediate = HPHt * H_grav.transpose();
        Eigen::Matrix3f S = S_intermediate + R_grav;
        // Apply Mahalanobis Gating
        Eigen::LDLT<Eigen::Matrix3f> ldlt_solver(S);
        Eigen::Vector3f S_inv_y = ldlt_solver.solve(y_grav);
        float d2 = y_grav.dot(S_inv_y);
        if(d2 < 16.0f) {
            pr_used = true;
            Eigen::Matrix<float,15,3> PHt = P * H_grav.transpose();
            Eigen::Matrix<float,3,15> PHt_T = PHt.transpose();
            Eigen::Matrix<float,3,15> K_T;
            for (int col = 0; col < 15; ++col) {
                K_T.col(col) = ldlt_solver.solve(PHt_T.col(col));
            }
            Eigen::Matrix<float,15,3> K = K_T.transpose();
            K.block<3,1>(9,0).setZero();   // acc_b
            P = (Eigen::Matrix<float,15,15>::Identity()-K*H_grav)*P*(Eigen::Matrix<float,15,15>::Identity()-K*H_grav).transpose() + K*R_grav*K.transpose();
            Eigen::Matrix<float,15,1> error_pred = K * y_grav;
            nom_state.p = nom_state.p + error_pred.segment<3>(0);
            nom_state.vel = nom_state.vel + error_pred.segment<3>(3);
            d_theta_grav = error_pred.segment<3>(6);
            d_theta_grav(2) = 0.0f;  // remove yaw
            if (d_theta_grav.norm() > 1.0f) {
            d_theta_grav = d_theta_grav.normalized();
            }
            gyro_grav = error_pred.segment<3>(12);
            gyro_grav(2) = 0.0f;
        }
        
    }


    if(new_mag && speed_norm < 1.0f) {
        Eigen::Matrix<float,1,15> H_mag;
        H_mag.setZero();
        filtered_mag_vec = mag_vec_alpha*mag_vec + (1-mag_vec_alpha)*filtered_mag_vec;
        Eigen::Matrix<float,1,3> yaw_grad = Eigen::Matrix<float,1,3>::Zero();
        float xy = pow(filtered_mag_vec(0),2)+pow(filtered_mag_vec(1),2);
        // float horiz_min = pow((0.01f*9.80665f),2);
        yaw_grad(0,0) = -filtered_mag_vec(1)/xy;
        yaw_grad(0,1) = filtered_mag_vec(0)/xy;
        Eigen::Matrix3f rot_der = skewSymmetric(filtered_mag_vec);
        H_mag.block<1,3>(0,6) = -yaw_grad*rot_der;
        Eigen::Matrix<float,1,1> V_mag = Eigen::Matrix<float,1,1>::Constant(mag_noise);

        
        // Calculate Magnetometer Kalman Gain
        float S = (H_mag * P * H_mag.transpose())(0,0) + V_mag(0,0);
        Eigen::Matrix<float,15,1> K_mag = P * H_mag.transpose() / S;
        // Zero out parts of K that stop from updating pitch,roll, xy gyro bias
        K_mag.block<3,1>(0,0).setZero();  // Position
        K_mag.block<3,1>(3,0).setZero();  // Velocity
        K_mag.block<3,1>(9,0).setZero();   // acc_b
        K_mag(6) = 0;
        K_mag(7) = 0;
        // Calculate observed error
        float est_heading = atan2f(
            2.0f * (nom_state.q.w() * nom_state.q.z() + nom_state.q.x() * nom_state.q.y()),
            1.0f - 2.0f * (nom_state.q.y() * nom_state.q.y() + nom_state.q.z() * nom_state.q.z())
        );
        yaw_pred = est_heading;
        // Calculate residual, ensure its wrapped between -pi,pi
        float y_mag = wrap_to_pi(mag_heading - est_heading);
        if (y_mag > M_PI)  y_mag -= 2.0f * M_PI;
        if (y_mag < -M_PI) y_mag += 2.0f * M_PI;
        float d2 = y_mag*y_mag / S;
        // float angle_error = fabsf(mag_heading-est_heading);
        if(d2 < 25.0f) {
            mag_used = true;
            Eigen::Matrix<float,15,1> error_pred = K_mag*(y_mag);
            // Update Covariance matrix
            P = (Eigen::Matrix<float,15,15>::Identity()-K_mag*H_mag)*P*(Eigen::Matrix<float,15,15>::Identity()-K_mag*H_mag).transpose() + K_mag*V_mag*K_mag.transpose();
            // Inject observed error into state
            d_theta_mag = error_pred.segment<3>(6);
            if (d_theta_mag.norm() > 1.0f) {
                d_theta_mag = d_theta_mag.normalized();
            }
            d_theta_mag(0) = 0.0f;  // remove roll
            d_theta_mag(1) = 0.0f;  // remove pitch
            gyro_mag = error_pred.segment<3>(12);
            gyro_mag(0) = 0.0f;
            gyro_mag(1) = 0.0f;
        }
    }
    // Check if there's new GPS and if the yaw is consistent
    if(new_gps && speed_norm > 0.75f && gps_cAcc < 0.2f) {
        gps_yaw_used = true;
        Eigen::Matrix<float,1,15> H_gps_yaw = Eigen::Matrix<float,1,15>::Zero();
        H_gps_yaw(0,8) = 1.0f;
        Eigen::Matrix<float,1,1> V_gps_yaw =  Eigen::Matrix<float,1,1>::Constant(gps_cAcc*gps_cAcc);
        // Calculate the Kalman gain of the barometer
        float S = (H_gps_yaw * P * H_gps_yaw.transpose())(0,0) + V_gps_yaw(0,0);
        Eigen::Matrix<float,15,1> K_gps_yaw = P * H_gps_yaw.transpose() / S;
        float y_gps_yaw = wrap_to_pi(gps_heading-get_yaw());
            Eigen::Matrix<float,15,1> error_pred = K_gps_yaw*(y_gps_yaw);
            // Update Covariance matrix
            P = (Eigen::Matrix<float,15,15>::Identity()-K_gps_yaw*H_gps_yaw)*P*(Eigen::Matrix<float,15,15>::Identity()-K_gps_yaw*H_gps_yaw).transpose() + K_gps_yaw*V_gps_yaw*K_gps_yaw.transpose();
            d_theta_gps = error_pred.segment<3>(6);
            if (d_theta_gps.norm() > 1.0f) {
                d_theta_gps = d_theta_mag.normalized();
            }
            d_theta_gps(0) = 0.0f;  // remove roll
            d_theta_gps(1) = 0.0f;  // remove pitch
            gyro_gps = error_pred.segment<3>(12);
            gyro_gps(0) = 0.0f;
            gyro_gps(1) = 0.0f;
    }
    else {
        gps_yaw_used = false;
    }

    // Since quaternion's aren't communitive, I've combined the grav vector error and the observed magnetometer error here and done one rotation
    if(d_theta_mag.norm() > 0 || d_theta_grav.norm() > 0 || d_theta_gps.norm() > 0) {
        Eigen::Quaternionf dq;
        Eigen::Vector3f d_theta_tot = d_theta_mag + d_theta_grav;
        float theta = d_theta_tot.norm();
        if (theta > 1e-5f) {
            dq = Eigen::AngleAxisf(theta, d_theta_tot.normalized());
        } else {
            dq = Eigen::Quaternionf(1, d_theta_tot(0)/2.0f, d_theta_tot(1)/2.0f, d_theta_tot(2)/2.0f);  // first-order
        }
        nom_state.q = (dq * nom_state.q).normalized();
        nom_state.gyro_b = nom_state.gyro_b + gyro_grav + gyro_mag;
    }
    if(d_theta_grav.norm() == 0) {
        // Inflate covariance matrix to reflect loss of confidence (we didn't obeserve anything this state)
        P(6,6) *= pr_cov_infl;
        P(7,7) *= pr_cov_infl;
        pr_used = false;
    }
    if(d_theta_mag.norm() == 0) {
        // Inflate covariance matrix to reflect loss of confidence (we didn't obeserve anything this state)
        P(8,8) *= yaw_cov_infl;
        mag_used = false;
    }


    if(new_gps) {
        Eigen::Matrix<float,6,15> H_gps;
        H_gps.setZero();
        H_gps.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        H_gps.block<3,3>(3,3) = Eigen::Matrix3f::Identity();
        Eigen::Matrix<float,6,6> V_gps = Eigen::Matrix<float,6,6>::Zero();
        V_gps(0,0) = fmax(1.0f,gps_hAcc*gps_hAcc);
        V_gps(1,1) = fmax(1.0f,gps_hAcc*gps_hAcc);
        V_gps(2,2) = fmax(4.0f,gps_vAcc*gps_vAcc);
        V_gps(3,3) = gps_sAcc*gps_sAcc;
        V_gps(4,4) = gps_sAcc*gps_sAcc;
        V_gps(5,5) = gps_sAcc*gps_sAcc;
        // Calculate GPS Kalman Gain
        Eigen::Matrix<float,15,6> K_gps = P * H_gps.transpose() * (H_gps*P*H_gps.transpose()+V_gps).ldlt().solve(Eigen::Matrix<float,6,6>::Identity());
        Eigen::Matrix<float,6,1> y_gps;
        y_gps.segment<3>(0) = gps_meas.segment<3>(0) - nom_state.p;
        y_gps.segment<3>(3) = gps_meas.segment<3>(3) - nom_state.vel;
        // Calculate observed error
        Eigen::Matrix<float,15,1> error_pred = K_gps*(y_gps);
        // // Update Covariance matrix
        P = (Eigen::Matrix<float,15,15>::Identity()-K_gps*H_gps)*P*(Eigen::Matrix<float,15,15>::Identity()-K_gps*H_gps).transpose() + K_gps*V_gps*K_gps.transpose();
        // Inject observed error into state
        nom_state.p = nom_state.p + error_pred.segment<3>(0);
        nom_state.vel = nom_state.vel + error_pred.segment<3>(3);
        Eigen::Vector3f d_theta = error_pred.segment<3>(6);
        Eigen::Quaternionf dq;
        float theta = d_theta.norm();
        if (theta > 1e-5f) {
            dq = Eigen::AngleAxisf(theta, d_theta.normalized());
        } else {
            dq = Eigen::Quaternionf(1, d_theta(0)/2.0f, d_theta(1)/2.0f, d_theta(2)/2.0f);  // first-order
        }
        nom_state.q = (dq * nom_state.q).normalized();
        nom_state.acc_b = nom_state.acc_b + error_pred.segment<3>(9);
        nom_state.gyro_b = nom_state.gyro_b + error_pred.segment<3>(12);
    }

    if(new_baro) {
        Eigen::Matrix<float,1,15> H_baro = Eigen::Matrix<float,1,15>::Zero();
        H_baro(0,2) = 1.0f;
        Eigen::Matrix<float,1,1> V_baro =  Eigen::Matrix<float,1,1>::Constant(baro_noise);
        // Calculate the Kalman gain of the barometer
        float S = (H_baro * P * H_baro.transpose())(0,0) + V_baro(0,0);
        Eigen::Matrix<float,15,1> K_baro = P * H_baro.transpose() / S;
        float y_baro = baro_meas-nom_state.p(2);
            // Calculate our observed error of the barometer
            Eigen::Matrix<float,15,1> error_pred = K_baro*(y_baro);
            // Update Covariance matrix
            P = (Eigen::Matrix<float,15,15>::Identity()-K_baro*H_baro)*P*(Eigen::Matrix<float,15,15>::Identity()-K_baro*H_baro).transpose() + K_baro*V_baro*K_baro.transpose();
            // Inject observed error into the nominal state
            nom_state.p = nom_state.p + error_pred.segment<3>(0);
            nom_state.vel = nom_state.vel + error_pred.segment<3>(3);
            Eigen::Vector3f d_theta = error_pred.segment<3>(6);
            Eigen::Quaternionf dq;
            float theta = d_theta.norm();
            if (theta > 1e-5f) {
                dq = Eigen::AngleAxisf(theta, d_theta.normalized());
            } else {
                dq = Eigen::Quaternionf(1, d_theta(0)/2.0f, d_theta(1)/2.0f, d_theta(2)/2.0f);  // first-order
            }
            nom_state.q = (dq * nom_state.q).normalized();
            nom_state.acc_b = nom_state.acc_b + error_pred.segment<3>(9);
            nom_state.gyro_b = nom_state.gyro_b + error_pred.segment<3>(12);
    }
    

    if (!P.allFinite()) {
        P.setIdentity();
        std::cerr << "WARNING: P corrupted — resetting!\n";
        return false;
    }
    return true;
}

// GETTERS:
float eskf::get_pitch() {
    float w = nom_state.q.w();
    float x = nom_state.q.x();
    float y = nom_state.q.y();
    float z = nom_state.q.z();
    return asinf(2*(w*y - z*x));
}

float eskf::get_roll() {
    float w = nom_state.q.w();
    float x = nom_state.q.x();
    float y = nom_state.q.y();
    float z = nom_state.q.z();
    return atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y)); 

}

float eskf::get_yaw() {
    float w = nom_state.q.w();
    float x = nom_state.q.x();
    float y = nom_state.q.y();
    float z = nom_state.q.z();
    return atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
}

// HELPERS:
Eigen::Matrix3f eskf::skewSymmetric(const Eigen::Vector3f& v) {
    Eigen::Matrix3f m;
    m.setZero();
    m <<     0.0f, -v.z(),  v.y(),
           v.z(),   0.0f, -v.x(),
          -v.y(),  v.x(),   0.0f;
    return m;
}


float eskf::wrap_to_pi(float angle) {
    angle = fmod(angle + M_PI, 2.0f * M_PI);
    if (angle < 0) angle += 2.0f * M_PI;
    return angle - M_PI;
}