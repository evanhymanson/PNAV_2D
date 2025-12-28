#include <iostream>
#include <cmath>
#include <vector>
#include "pnav.h"

using namespace std;

// Target method implementations
void Target::init(double x0, double y0, double v0, double a0, double gam0) {
    x = x0;
    y = y0;
    v = v0;
    a = a0;
    gam = gam0;
    gamd = a / v;
    vx = -v*cos(gam);
    vy = -v*sin(gam);
}

void Target::update(double dt) {
    double x_old = x;
    double y_old = y;
    double gam_old = gam;
    double vx_old = vx;
    double vy_old = vy;

    // PREDICTOR STEP (Euler)
    double gam_pred = gam_old + dt * gamd;
    double vx_pred = -v * cos(gam_pred);
    double vy_pred = -v * sin(gam_pred);

    // CORRECTOR STEP (average derivatives)
    gam = gam_old + dt * gamd;
    x = x_old + dt * 0.5 * (vx_old + vx_pred);
    y = y_old + dt * 0.5 * (vy_old + vy_pred);

    // Update velocities from final gam
    vx = -v * cos(gam);
    vy = -v * sin(gam);
}

// Missile method implementations
void Missile::init(double x0, double y0, double v0, double hd0, double ax0, double ay0) {
    x = x0;
    y = y0;
    v = v0;
    hd = hd0 / 57.3;
    ax = ax0;
    ay = ay0;
}

void Missile::compute_guide(double N, double vc, double xlamd, double xlam) {
    double a_cmd = N * vc * xlamd;
    ax_cmd = - a_cmd*sin(xlam);
    ay_cmd = a_cmd*cos(xlam);
}

void Missile::update(double dt) {
    // store old values 
    double x_old = x;
    double y_old = y;
    double vx_old = vx;
    double vy_old = vy;
    
    // predict 
    double x_pred = x_old + vx_old*dt;
    double y_pred = y_old + vy_old*dt;
    double vx_pred = vx_old + ax_cmd*dt;
    double vy_pred = vy_old + ay_cmd*dt;

    // corrector 
    x = x_old + 0.5*dt*(vx_old + vx_pred);
    y = y_old + 0.5*dt*(vy_old + vy_pred);
    vx = vx_old + 0.5*dt*(ax_cmd + ax_cmd);
    vy = vy_old + 0.5*dt*(ay_cmd + ay_cmd);
}

RelativeState computeRelative(const Missile& missile, const Target& targ) {
    RelativeState rel;

    rel.x = targ.x - missile.x;
    rel.y = targ.y - missile.y;
    rel.vx = targ.vx - missile.vx;
    rel.vy = targ.vy - missile.vy;
    rel.r = sqrt(rel.x*rel.x + rel.y*rel.y);

    rel.xlam = atan2(rel.y, rel.x); // los 
    rel.xlamd = (rel.x*rel.vy - rel.y*rel.vx) / (rel.r*rel.r); // d (los)

    rel.vc = -(rel.x*rel.vx + rel.y*rel.vy) / rel.r;

    return rel;
}   

void SimulatePNav2d(Missile& missile, Target& targ) {

    RelativeState rel = computeRelative(missile, targ);
    
    double xlead = asin(targ.v * sin(targ.gam - rel.xlam) / missile.v);
    double theta = rel.xlam + xlead;

    missile.vx = missile.v * cos(theta + missile.hd);
    missile.vy = missile.v * sin(theta + missile.hd);

    double t = 0.0;
    double h = 0.0;

    const double N = 4.0;

    // Engagement Loop
    while (rel.vc > 0.0) {

        if (rel.r > 10) {
            h = .001;
        } else {
            h = .00001;
        }

        // Relative geometry
        rel = computeRelative(missile, targ);

        // Guidance law
        missile.compute_guide(N, rel.vc, rel.xlamd, rel.xlam);

        // Dynamics
        missile.update(h);
        targ.update(h);

        cout << "t: " << t << ", r: " << rel.r << endl;

        t += h;

    }
}

