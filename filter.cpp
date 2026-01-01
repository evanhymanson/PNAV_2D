#include <iostream>
#include "pnav.h"
#include "filter.h"
#include <cmath>

void Digital_Fading_Memory_Filter::filter(double xlam) {

    // Add noise to measured value
    double xlam_meas = xlam + noise();

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

double Digital_Fading_Memory_Filter::noise() {
    return sigma_noise * distribution(generator);
}
double Digital_Fading_Memory_Filter::get_xh_xlam() {
    return xh_lam;
}

double Digital_Fading_Memory_Filter::get_xh_xlamd() {
    return xh_lamd;
}