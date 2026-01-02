#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#include <vector>
#include "pnav.h"
#include <cmath>
#include <random>
#include "sensor_noise.h"
#include <Eigen/Dense>
class Digital_Fading_Memory_Filter {
public: 
    double beta;
    double G;
    double H; 
    double xh_lam;
    double xh_lamd;
    double xh_lam_prev;
    double xh_lamd_prev;
    double ts; // sampling time

    Digital_Fading_Memory_Filter(double beta, double xlam_init, double xlamd_init): beta(beta), G(1-pow(beta,2)), H(pow(1-beta,2)), xh_lam(xlam_init), xh_lamd(xlamd_init), ts(0.05), xh_lam_prev(xlam_init), xh_lamd_prev(xlamd_init) {};

    void filter(double xlam);
    double get_xh_xlamd();
    double get_xh_xlam();
};

class Kalman_Filter {
    private: 
        Eigen::Vector2d x; 
        Eigen::Matrix2d Phi;
        Eigen::Matrix2d P;
        Eigen::Matrix2d Q;
        Eigen::RowVector2d H;
        double R;

        void buildPhi();
        void buildQ(double process_noise);

    public: 
        double ts;

        Kalman_Filter(double ts, double process_noise, double xlam_init, double xlamd_init, double meas_noise) : ts(ts), R(meas_noise*meas_noise) {
            
            // Initial state vector
            x << xlam_init, xlamd_init;

            // Inital measurement matrix: H = [1,0]
            H << 1.0, 0.0;

            // State transition matrix 
            buildPhi();

            // Build process noise covariance
            buildQ(process_noise);

            // Initalize error covariance 
            P << 1.0, 0.0,
                0.0, 1.0;
        }

        void filter(double xlam_meas);

        // Getters
        double get_xh_xlam() const;
        double get_xh_xlamd() const;

};

#endif