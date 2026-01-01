#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#include <vector>
#include "pnav.h"
#include <cmath>
#include <random>

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

    // Gaussian distribution
    std::normal_distribution<double> distribution; // 0 to 1
    double sigma_noise;
    std::default_random_engine generator;

    Digital_Fading_Memory_Filter(double beta, double xlam_init, double xlamd_init, double signoise): beta(beta), G(1-pow(beta,2)), H(pow(1-beta,2)), xh_lam(xlam_init), xh_lamd(xlamd_init), ts(0.05), xh_lam_prev(xlam_init), xh_lamd_prev(xlamd_init), sigma_noise(signoise), distribution(0.0, 1.0) {
        std::random_device rd;
        generator.seed(rd());
    };

    void filter(double xlam);
    double noise();
    double get_xh_xlamd();
    double get_xh_xlam();
};

#endif