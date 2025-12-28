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
    void update(double dt);
};

class Missile {
public:
    double x, y;
    double vx, vy;
    double v;
    double ax, ay;
    double ax_cmd, ay_cmd;
    double hd;

    void init(double x0, double y0, double v0, double hd0, double ax0, double ay0);
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

void SimulatePNav2d(Missile& missile, Target& targ);
RelativeState computeRelative(const Missile& missile, const Target& targ);

#endif