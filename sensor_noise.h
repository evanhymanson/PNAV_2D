#ifndef NOISE_H
#define NOISE_H

#include <iostream>
#include <vector>
#include <cmath>
#include <random>

class SensorNoise {
public: 
    // Gaussian distribution
    std::normal_distribution<double> distribution; // 0 to 1
    double sigma_noise;
    std::default_random_engine generator;
    
    SensorNoise(double sigma_noise): sigma_noise(sigma_noise), distribution(0.0, 1.0) {
        std::random_device rd;
        generator.seed(rd());
    };

    double noise();
};

#endif