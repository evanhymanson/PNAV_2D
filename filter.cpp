#include <iostream>
#include "pnav.h"
#include "filter.h"
#include <cmath>
#include "sensor_noise.h"
#include <Eigen/Dense>

void Digital_Fading_Memory_Filter::filter(double xlam_meas) {

    // Predict 
    double xlam_pred = xh_lam_prev + xh_lamd_prev*ts;

    // Residual 
    double res = xlam_meas - xlam_pred;

    // Update 
    xh_lam = xlam_pred + G*res;
    xh_lamd = xh_lamd_prev + (H/ts)*res;

    // Save for next iteration 
    xh_lam_prev = xh_lam;
    xh_lamd_prev = xh_lamd;
}

double Digital_Fading_Memory_Filter::get_xh_xlam() {
    return xh_lam;
}

double Digital_Fading_Memory_Filter::get_xh_xlamd() {
    return xh_lamd;
}

void Kalman_Filter::buildPhi() {
    Phi << 1.0, ts,
            0.0, 1.0;
}

void Kalman_Filter::buildQ(double process_noise) {
    // continous-time to discrete-time process noise
    double q = process_noise * process_noise;
    Q << pow(ts,3)/3, pow(ts,2)/2,
        pow(ts,2)/2, ts;
    Q *= q;
}

void Kalman_Filter::filter(double xlam_meas) {

    // PREDICT:
    Eigen::Vector2d x_pred = Phi * x;
    Eigen::Matrix2d P_pred = Phi * P * Phi.transpose() + Q;

    // UPDATE:
    // Innovation Residual
    double y = xlam_meas - H * x_pred;

    // Innovation Covariance
    double S = H * P_pred * H.transpose() + R;

    // Kalman Gain
    Eigen::Vector2d K = P_pred * H.transpose() / S;

    // Update state estimate
    x = x_pred + K*y;

    // Update covariance estimate 
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    P = (I - K*H) * P_pred;

}

double Kalman_Filter::get_xh_xlam() const {
    return x(0);
}

double Kalman_Filter::get_xh_xlamd() const {
    return x(1);
}