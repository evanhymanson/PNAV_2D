#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <vector>
#include "pnav.h"
#include "filter.h"

class Logger {
public: 
    std::vector<double> time;

    struct missile_hist {
        std::vector<double> x, y;
        std::vector<double> vx, vy;
        std::vector<double> ax_cmd, ay_cmd;
        std::vector<double> hd;  
    };

    missile_hist missile_hist;
    struct targ_hist {
        std::vector<double> x, y;
        std::vector<double> vx, vy;
        std::vector<double> v;
        std::vector<double> gam, gamd;
        std::vector<double> a;
    };

    struct filter_hist {
        std::vector<double> xlam, xlamd;
    };

    struct rel_hist {
        std::vector<double> x, y;       // relative position
        std::vector<double> vx, vy;   // relative velocity
        std::vector<double> r;                // range
        std::vector<double> xlam;             // line-of-sight angle
        std::vector<double> xlamd;            // LOS rate filtered
        // std::vector<double> xlamd_orig;       // orig from geometric calc
        std::vector<double> vc;               // closing velocity
    };

    rel_hist rel_hist;
    targ_hist targ_hist;
    filter_hist filter_hist;
    
    void log(double t, const Missile& missile, const Target& targ, const RelativeState& rel, const Digital_Fading_Memory_Filter& filter);

    void exportCSV(const std::string& filename) const;
};

#endif
