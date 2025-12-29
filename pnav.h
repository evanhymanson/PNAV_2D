#ifndef PNAV_H
#define PNAV_H

#include <vector>

class Target {
public:
    double x, y;
    double vx, vy;
    double gam, gamd;
    double v;
    double a;

    void init(double x0, double y0, double v0, double a0, double gam0);
    void update(double dt, double t);
};

class Missile {
public:
    double x, y;
    double vx, vy;
    double v;
    double ax_cmd, ay_cmd;
    double hd;

    void init(double x0, double y0, double v0, double hd0);
    void compute_guide(double N, double vc, double xlamd, double xlam);
    void update(double dt);
};

struct RelativeState {
    double x, y;       // relative position
    double vx, vy;   // relative velocity
    double r;                // range
    double xlam;             // line-of-sight angle
    double xlamd;            // LOS rate
    double vc;               // closing velocity
};

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

    targ_hist targ_hist;
    struct rel_hist {
        std::vector<double> x, y;       // relative position
        std::vector<double> vx, vy;   // relative velocity
        std::vector<double> r;                // range
        std::vector<double> xlam;             // line-of-sight angle
        std::vector<double> xlamd;            // LOS rate
        std::vector<double> vc;               // closing velocity
    };

    rel_hist rel_hist;

    void log(double t, const Missile& missile, const Target& targ, const RelativeState& rel);
};

void SimulatePNav2d(Missile& missile, Target& targ);
RelativeState computeRelative(const Missile& missile, const Target& targ);

#endif