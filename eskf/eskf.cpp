#include "eskf.h"

// From IMU Datasheet:
/*
    accelerometer noise, gyro noise
*/

// Perturbation impulses vector
eskf::eskf() {
    // Set start state
    deltaT = 0.05;
    g = Eigen::Vector3f(0.0f,0.0f,9.81f);
    B_ned = Eigen::Vector3f(0.4184f,-0.0791f,0.9048f);
    // TODO: Allow setting the Noise vars via the constructor (currently done in class def)
    // Initialize equal covariances to equal weighting
    P = Eigen::Matrix<float,15,15>::Identity();
}

bool eskf::update(bool new_baro, bool new_gps, bool new_mag) {
    // TODO: Update acc_meas, gyro_meas
    predict_state();
    fuse_sensors(new_mag, new_baro, new_gps);

    return true;
}
        
void eskf::predict_state() {

    // Predict state x_k+1
    nom_state.q.normalize();
    Eigen::Matrix3f R = nom_state.q.toRotationMatrix();

    // Predict nominal State (x)
    // Nominal State Kinematics
    acc_ned = R * (acc_meas - nom_state.acc_b) - g;
    // acc_ned = R * (acc_meas - nom_state.acc_b) + Eigen::Vector3f(0, 0, 9.81);

    nom_state.p = nom_state.p + nom_state.vel*deltaT + R*acc_ned*0.5f*deltaT*deltaT;
    nom_state.vel = nom_state.vel + acc_ned*deltaT;
    nom_state.acc_b = nom_state.acc_b;
    nom_state.gyro_b = nom_state.gyro_b;

    Eigen::Vector3f omega = gyro_meas - nom_state.gyro_b;
    float angle = omega.norm() * deltaT;
    Eigen::Vector3f axis = (angle < 1e-5f) ? omega : omega.normalized();
    Eigen::Quaternionf dq(Eigen::AngleAxisf(angle, axis));
    nom_state.q = (nom_state.q * dq).normalized();
    
    Eigen::Vector3f acc_diff = acc_meas - nom_state.acc_b;
    Eigen::Matrix3f acc_skew = skewSymmetric(acc_diff);

    predict_covariance(R,acc_skew);
}

void eskf::predict_covariance(Eigen::Matrix3f R, Eigen::Matrix3f acc_skew) {

    // Calculate error State Jacobian (F_x)
    Eigen::Matrix<float, 15, 15> Fx;
    Fx.setZero();
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

// TODO: Setup system where barometer and GPS measurements are updated continuously, once gotten call this fn
// TODO: Continuously update the GPS accuracy with the signal
// Note: Both the GPS and Barometer have linear sensor models. Therefore we don't need to get super fancy with the jacobian math here
eskf::State eskf::fuse_sensors(bool new_mag, bool new_baro, bool new_gps) {

    /*
        Currently GPS horizontal positon, GPS vertical position, and barometer vertical position are implemented
        TODO: GPS Velocity (Is there even error?), Magnetometer (Bomboclat, might have to add in a)
    
    */

    if(new_gps) {
        Eigen::Matrix<float,3,15> H_gps;
        H_gps.setZero();
        H_gps.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        Eigen::Matrix<float,3,3> V_gps = Eigen::Matrix<float,3,3>::Zero();
        // TODO: Model the accuracies as an estimated 1-sigma standard dev, ensure unit is meters
        V_gps(0,0) = std::max(gps_hAcc*gps_hAcc, 1.0f);  // at least 1 m²
        V_gps(1,1) = std::max(gps_hAcc*gps_hAcc, 1.0f);
        V_gps(2,2) = std::max(gps_vAcc*gps_vAcc, 2.0f);  // z often noisier
        // Calculate GPS Kalman Gain
        Eigen::Matrix<float,15,3> K_gps = P * H_gps.transpose() * (H_gps*P*H_gps.transpose()+V_gps).ldlt().solve(Eigen::Matrix<float,3,3>::Identity());
        K_gps.block<3,3>(6,0).setZero();  // orientation
        K_gps.block<3,3>(9,0).setZero();   // acc_b
        K_gps.block<3,3>(12,0).setZero();  // gyro_b
        // std::cout << K_gps.norm() << "\n";
        Eigen::Vector3f y_gps(gps_x,gps_y,gps_z);
        // Calculate observed error
        error_pred = K_gps*(y_gps-nom_state.p);
        // Update Covariance matrix
        P = (Eigen::Matrix<float,15,15>::Identity()-K_gps*H_gps)*P*(Eigen::Matrix<float,15,15>::Identity()-K_gps*H_gps).transpose() + K_gps*V_gps*K_gps.transpose();
        // TODO: Make sure covariance matrix is positive definite (with pos eigenvalues, symmetric)

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

    // TODO: Implement something to not use barometer when it sucks (ie when GPS height is less than 1 don't use baro)
    if(new_baro) {
        Eigen::Matrix<float,1,15> H_baro = Eigen::Matrix<float,1,15>::Zero();
        H_baro(0,2) = 1.0f;
        Eigen::Matrix<float,1,1> V_baro =  Eigen::Matrix<float,1,1>::Constant(baro_noise);
        // Calculate the Kalman gain of the barometer
        float S = (H_baro * P * H_baro.transpose())(0,0) + V_baro(0,0);
        Eigen::Matrix<float,15,1> K_baro = P * H_baro.transpose() / S;
        K_baro.segment<3>(12).setZero();
        // TODO: Figure out what Kalman gains to zero out from ekf2 (probably everything x,y related, q, gyro)
        float y_baro = baro_meas-nom_state.p(2);
        // Make sure that the barometer reading is in reasonable error range, if not don't use it (Disabled for now)
        // if((S>V_baro(0,0))&& fabs(y_baro) < baro_dev_tol*sqrtf(S)) {
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
        // }
    }

    bool used_grav = false;
    Eigen::Vector3f d_theta_mag = Eigen::Vector3f::Zero();
    Eigen::Vector3f d_theta_grav = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_grav = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_mag = Eigen::Vector3f::Zero();

    // TODO: Implement a measure to not use the magnetometer when it sucks
    if(new_mag) {
        Eigen::Matrix<float,1,15> H_mag;
        H_mag.setZero();
        Eigen::Matrix<float,1,3> yaw_grad = Eigen::Matrix<float,1,3>::Zero();
        // Eigen::Vector3f mag_ned = nom_state.q * mag_vec;
        float xy = pow(mag_vec(0),2)+pow(mag_vec(1),2);
        float horiz_min = pow((0.01f*9.80665f),2);
        yaw_grad(0,0) = -mag_vec(1)/xy;
        yaw_grad(0,1) = mag_vec(0)/xy;
        Eigen::Matrix3f rot_der = skewSymmetric(mag_vec);
        H_mag.block<1,3>(0,6) = -yaw_grad*rot_der;
        Eigen::Matrix<float,1,1> V_mag = Eigen::Matrix<float,1,1>::Constant(mag_noise);

        
        // Calculate Magnetometer Kalman Gain
        float S = (H_mag * P * H_mag.transpose())(0,0) + V_mag(0,0);
        Eigen::Matrix<float,15,1> K_mag = P * H_mag.transpose() / S;
        // Zero out parts of K that stop from updating pitch,roll, xy gyro bias
        K_mag.block<3,1>(0,0).setZero();  // Position
        K_mag.block<3,1>(3,0).setZero();  // Velocity
        K_mag.block<3,1>(9,0).setZero();   // acc_b
        // Calculate observed error
        float est_heading = atan2f(
            2.0f * (nom_state.q.w() * nom_state.q.z() + nom_state.q.x() * nom_state.q.y()),
            1.0f - 2.0f * (nom_state.q.y() * nom_state.q.y() + nom_state.q.z() * nom_state.q.z())
        );
        // Calculate residual, ensure its wrapped between 0,2pi
        float y_mag = wrapToPi(mag_heading - est_heading);
        if (y_mag > M_PI)  y_mag -= 2.0f * M_PI;
        if (y_mag < -M_PI) y_mag += 2.0f * M_PI;
        y_yaw = y_mag;
        float d2 = y_mag*y_mag / S;
        float angle_error = fabsf(mag_heading-est_heading);
        std::cout << mag_heading << "," << est_heading << ",";
        if(xy >= horiz_min && d2 < 9.0f && angle_error < 0.15f) {
            // std::cout << "Update yaw" << "\n";
            Eigen::Matrix<float,15,1> error_pred = K_mag*(y_mag);
            // Update Covariance matrix
            P = (Eigen::Matrix<float,15,15>::Identity()-K_mag*H_mag)*P*(Eigen::Matrix<float,15,15>::Identity()-K_mag*H_mag).transpose() + K_mag*V_mag*K_mag.transpose();
            // TODO: Make sure covariance matrix is positive definite (with pos eigenvalues, symmetric)

            // Inject observed error into state
            Eigen::Vector3f d_theta_mag = error_pred.segment<3>(6);
                    if (d_theta_mag.norm() > 1.0f) {
                d_theta_mag = d_theta_mag.normalized();
            }
            d_theta_mag(0) = 0.0f;  // remove roll
            d_theta_mag(1) = 0.0f;  // remove pitch
            std::cout << d_theta_mag(2);
            gyro_mag = error_pred.segment<3>(12);
            gyro_mag(0) = 0.0f;
            gyro_mag(1) = 0.0f;
        }
        std::cout << "\n";
    }


    // Full Vector Approximation Model
    // if(new_mag) {
    //     Eigen::Matrix<float,3,15> H_mag;
    //     H_mag.setZero();
    //     Eigen::Matrix3f V_mag = Eigen::Matrix3f::Identity() * mag_noise;
    //     Eigen::Matrix3f R_q_t = nom_state.q.toRotationMatrix().transpose();
    //     Eigen::Matrix3f skew_bn = skewSymmetric(B_ned);
    //     H_mag.block<3,3>(0,6) = R_q_t * skew_bn;
    //     // Calculate residual
    //     Eigen::Vector3f m_pred = R_q_t * B_ned;
    //     Eigen::Vector3f y_mag = mag_vec.normalized() - m_pred.normalized();
    //     // std::cout << "Update yaw" << "\n";
    //     // Calculate and Apply Kalman gain
    //     Eigen::Matrix3f S = H_mag * P * H_mag.transpose() + V_mag;
    //     Eigen::Matrix<float,15,3> K_mag = P * H_mag.transpose()*S.inverse();  
    //     Eigen::Matrix<float,15,1> error_pred = K_mag*(y_mag);
    //     // Update Covariance matrix
    //     P = (Eigen::Matrix<float,15,15>::Identity()-K_mag*H_mag)*P*(Eigen::Matrix<float,15,15>::Identity()-K_mag*H_mag).transpose() + K_mag*V_mag*K_mag.transpose();
    //     // TODO: Make sure covariance matrix is positive definite (with pos eigenvalues, symmetric)

    //     // Inject observed error into state
    //     Eigen::Vector3f d_theta_mag = error_pred.segment<3>(6);
    //             if (d_theta_mag.norm() > 0.2f) {
    //         d_theta_mag = d_theta_mag.normalized()*0.02f;
    //     }
    //     d_theta_mag(0) = 0.0f;
    //     d_theta_mag(1) = 0.0f;
    //     gyro_mag = error_pred.segment<3>(12);
    //     gyro_mag(0) = 0.0f;
    //     gyro_mag(1) = 0.0f;
    //     std::cout << d_theta_mag(2);
    //     std::cout << "\n";
    // }




    // Use the measured gravity to correct pitch and roll:
    float acc_norm = (acc_meas - nom_state.acc_b).norm();
    // std::cout << acc_norm << "\n";
    
    if (acc_norm > 9.0f && acc_norm < 10.5f) {
        used_grav = true;
        // std::cout << "Correcting Pitch Roll" << "\n";
        nom_state.q.normalize();
        Eigen::Matrix3f R = nom_state.q.toRotationMatrix();
        Eigen::Vector3f grav_pred = R.transpose()*g;
        // std::cout << grav_pred(2);
        Eigen::Matrix<float,3,15> H_grav = Eigen::Matrix<float,3,15>::Zero();
        H_grav.block<3,3>(0,6) = skewSymmetric(grav_pred);
        Eigen::Vector3f grav_meas = (acc_meas - nom_state.acc_b).normalized() * 9.81f;
        // std::cout << " " << grav_meas(2) << "\n";
        Eigen::Matrix3f R_grav = predicted_grav_noise * Eigen::Matrix3f::Identity();

        Eigen::Vector3f y_grav = grav_meas-grav_pred;
        // std::cout << y_grav.norm() << "\n";
        Eigen::Matrix3f S = H_grav * P * H_grav.transpose() + R_grav;
        // Apply Mahalanobis Gating
        float d2 = y_grav.transpose()*S.inverse()*y_grav;
        float angle_error = acosf(grav_pred.normalized().dot(grav_meas.normalized()));
        // std::cout << d2 << "," << angle_error;
        if(d2 < 9.0f && angle_error < 0.2f) {
            // std::cout << "," << "update";
            Eigen::Matrix<float, 15, 3> K = P * H_grav.transpose() * S.inverse();
            error_pred = K * y_grav;
            nom_state.p = nom_state.p + error_pred.segment<3>(0);
            nom_state.vel = nom_state.vel + error_pred.segment<3>(3);
            d_theta_grav = error_pred.segment<3>(6);
            d_theta_grav(2) = 0.0f;  // remove yaw
            if (d_theta_grav.norm() > 0.1f) {
            d_theta_grav = d_theta_grav.normalized() * 0.1f;
        }
        }
        gyro_grav = error_pred.segment<3>(12);
        gyro_grav(2) = 0.0f;
        // std::cout << "\n";
    }

    // Since quaternion's aren't communitive, I've combined the grav vector error and the observed magnetometer error here and done one rotation
    if(new_mag || used_grav) {
        Eigen::Quaternionf dq;
        Eigen::Vector3f d_theta = d_theta_mag + d_theta_grav;
        float theta = d_theta.norm();
        if (theta > 1e-5f) {
            dq = Eigen::AngleAxisf(theta, d_theta.normalized());
        } else {
            dq = Eigen::Quaternionf(1, d_theta(0)/2.0f, d_theta(1)/2.0f, 0);  // first-order
        }
        nom_state.q = (dq * nom_state.q).normalized();
        // if(gyro_grav.norm() > 0.1f) {
        //     gyro_grav = gyro_grav.norm()*0.1f;
        // }
        // if(gyro_mag.norm() > 0.1f) {
        //     gyro_mag = gyro_mag.norm()*0.1f;
        // }
        nom_state.gyro_b = nom_state.gyro_b + gyro_grav + gyro_mag;
    }

    if (!P.allFinite()) {
        P.setIdentity();
        std::cerr << "WARNING: P corrupted — resetting!\n";
    }
    return nom_state;
}

// GETTERS:
float eskf::getPitch() {
    float w = nom_state.q.w();
    float x = nom_state.q.x();
    float y = nom_state.q.y();
    float z = nom_state.q.z();
    return asinf(2*(w*y - z*x));
}

float eskf::getRoll() {
    float w = nom_state.q.w();
    float x = nom_state.q.x();
    float y = nom_state.q.y();
    float z = nom_state.q.z();
    return atan2f(2*(w*x + y*z), 1 - 2*(x*x + y*y)); 

}

float eskf::getYaw() {
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
    m <<     0, -v.z(),  v.y(),
          v.z(),     0, -v.x(),
         -v.y(),  v.x(),     0;
    return m;
}

float eskf::wrapToPi(float angle) {
    angle = fmod(angle + M_PI, 2.0f * M_PI);
    if (angle < 0) angle += 2.0f * M_PI;
    return angle - M_PI;
}