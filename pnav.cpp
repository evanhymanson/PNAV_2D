#include <iostream>
#include <cmath>
#include <vector>
#include "pnav.h"
#include "logger.h"
#include "filter.h"
#include "sensor_noise.h"
#include <fstream>

using namespace std;

// Target method implementations
void Target::init(double x0, double y0, double v0, double a0, double gam0, Maneuver maneuver) {
    this->maneuver = maneuver;
    x = x0;
    y = y0;
    v = v0;
    this->a0 = a0;
    gam = gam0 * M_PI / 180.0;
    vx = -v*cos(gam);
    vy = -v*sin(gam);
    a = compute_lateral_accel(0);
}

double Target::compute_lateral_accel(double t) {
    switch(maneuver) {
        case Maneuver::CONSTANT_TURN:
            return a0;
            break;

        case Maneuver::WEAVE:
            return a0 * sin(10.0 * t);
            break;

        case Maneuver::BARREL_ROLL:
            return a0 * sin(4.0 * t);
            break;

        case Maneuver::SPLIT_S: 
            return (t > 30) ? -a0 : a0;
            break;

        case Maneuver::SPIRAL_DIVE:
            return a0 * (1.0 + 0.1*t);

        case Maneuver::BANG_BANG:
            return (fmod(t, 20.0) < 10.0) ? a0 : -a0;

        default:
            return a0;
            break;
    }
}
void Target::update(double dt, double t) {
    
    a = compute_lateral_accel(t);
    double gamd = a / v;
    double x_old = x;
    double y_old = y;
    double gam_old = gam;
    double gamd_old = gamd;
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
void Missile::init(double x0, double y0, double v0, double hd0, double tau) {
    x = x0;
    y = y0;
    v = v0;
    hd = hd0 * M_PI / 180.0;  // Convert degrees to radians
    this->tau = tau; // system time constant 
}

void Missile::PN(double N, double vc, double xlamd, double xlam) {
    double a_cmd = N * vc * xlamd;
    ax_cmd = -a_cmd*sin(xlam);
    ay_cmd = a_cmd*cos(xlam);
}

void Missile::APN(double N, double vc, double xlamd, double xlam, double targ_a) {
    double a_cmd = N * vc * xlamd + (N * targ_a) / 2;
    ax_cmd = -a_cmd*sin(xlam);
    ay_cmd = a_cmd*cos(xlam);
}

void Missile::update(double dt) {
    // first order lag: tau * a' + a = a_cmd
    // discrete: a_actual(k+1) = a_actual(k) + dt/tau*(a_cmd - a_actual(k))

    double alpha = dt / tau; // lag coef.

    // Update with lag 
    ax = ax + alpha * (ax_cmd - ax);
    ay = ay + alpha * (ay_cmd - ay);

    // store old values 
    double x_old = x;
    double y_old = y;
    double vx_old = vx;
    double vy_old = vy;
    
    // predict 
    double x_pred = x_old + vx_old*dt;
    double y_pred = y_old + vy_old*dt;
    double vx_pred = vx_old + ax*dt;
    double vy_pred = vy_old + ay*dt;

    // corrector 
    x = x_old + 0.5*dt*(vx_old + vx_pred);
    y = y_old + 0.5*dt*(vy_old + vy_pred);
    vx = vx_old + 0.5*dt*(ax + ax);
    vy = vy_old + 0.5*dt*(ay + ay);

    hd = atan2(vy, vx);
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

    // Initialize missile velocities first (need these for computeRelative)
    RelativeState rel_temp = computeRelative(missile, targ);
    double xlead = asin(targ.v * sin(targ.gam - rel_temp.xlam) / missile.v);
    double theta = rel_temp.xlam + xlead;

    missile.vx = missile.v * cos(theta + missile.hd);
    missile.vy = missile.v * sin(theta + missile.hd);

    // Now compute relative state with proper missile velocities
    RelativeState rel = computeRelative(missile, targ);

    double meas_sigma = .005;
    double q = 5.0;
    SensorNoise meas_noise(meas_sigma);
    Kalman_Filter filter(.05, q, rel.xlam, rel.xlamd, meas_sigma);
    filter.filter(rel.xlam + meas_noise.noise());

    double t = 0.0;
    double h = 0.0;
    double t_filter = 0.0;

    const double N = 4;

    Logger log;
    log.log(t, missile, targ, rel, filter);

    // Engagement Loop
    while (true) {

        if (rel.r > 10) {
            h = 1e-3;
        } else {
            h = 1e-4;
        }

        // Relative geometry
        rel = computeRelative(missile, targ);

        // Check termination condition
        if (rel.vc <= 0 || rel.r < 0.1) {
            break;
        }

        // Sample frequency
        if (t_filter >= filter.ts) {
            filter.filter(rel.xlam + meas_noise.noise());
            t_filter = 0;
        }

        // Get filtered values
        double xh_lam = filter.get_xh_xlam();
        double xh_lamd = filter.get_xh_xlamd();

        // Guidance law
        missile.APN(N, rel.vc, xh_lamd, xh_lam, targ.a);

        // Dynamics
        missile.update(h);
        targ.update(h, t);

        t += h;
        t_filter += h;

        log.log(t, missile, targ, rel, filter);

        cout << "t: " << t << ", r: " << rel.r << endl;

    }

    log.exportCSV("sim.csv");
}

