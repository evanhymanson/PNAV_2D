#include <iostream>
#include <random>
#include "sensor_noise.h"

double SensorNoise::noise() {
    return sigma_noise * distribution(generator);
}