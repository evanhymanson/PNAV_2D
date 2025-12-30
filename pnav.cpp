#include <iostream>
#include <cmath>
#include <vector>
#include "pnav.h"
#include <fstream>

using namespace std;

// Target method implementations
void Target::init(double x0, double y0, double v0, double a0, double gam0) {
    x = x0;
    y = y0;
    v = v0;
    a = a0;
    gam = gam0 * M_PI / 180.0;
    vx = -v*cos(gam);
    vy = -v*sin(gam);
}

void Target::update(double dt, double t) {

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
void Missile::init(double x0, double y0, double v0, double hd0) {
    x = x0;
    y = y0;
    v = v0;
    hd = hd0 * M_PI / 180.0;  // Convert degrees to radians
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

void Logger::log(double t, const Missile& missile, const Target& targ, const RelativeState& rel) {
    time.push_back(t);
    
    // missile_hist missile_hist;
    missile_hist.x.push_back(missile.x);
    missile_hist.y.push_back(missile.y);
    missile_hist.vx.push_back(missile.vx);
    missile_hist.vy.push_back(missile.vy);
    missile_hist.ax_cmd.push_back(missile.ax_cmd);
    missile_hist.ay_cmd.push_back(missile.ay_cmd);
    missile_hist.hd.push_back(missile.hd);

    // targ_hist targ_hist;
    targ_hist.x.push_back(targ.x);
    targ_hist.y.push_back(targ.y);
    targ_hist.v.push_back(targ.v);
    targ_hist.vx.push_back(targ.vx);
    targ_hist.vy.push_back(targ.vy);
    targ_hist.a.push_back(targ.a);
    targ_hist.gam.push_back(targ.gam);
    targ_hist.gamd.push_back(targ.gamd);

    // rel_hist rel_hist;
    rel_hist.x.push_back(rel.x);
    rel_hist.y.push_back(rel.y);
    rel_hist.vx.push_back(rel.vx);
    rel_hist.vy.push_back(rel.vy);
    rel_hist.r.push_back(rel.r);
    rel_hist.xlam.push_back(rel.xlam);
    rel_hist.xlamd.push_back(rel.xlamd);
    rel_hist.vc.push_back(rel.vc);
}

void Logger::exportCSV(const string& filename) const {
    ofstream file(filename);

    file << "time,m_x,m_y,m_vx,m_vy,m_ax_cmd,m_ay_cmd,m_hd,";
    file << "t_x,t_y,t_vx,t_vy,t_v,t_a,t_gam,t_gamd,";
    file << "rel_x,rel_y,rel_r,rel_vx,rel_vy,rel_xlam,rel_xlamd,rel_vc\n";

    for (int i = 0; i < time.size(); i++) {
        file << time[i] << ",";
        file << missile_hist.x[i] << "," << missile_hist.y[i] << ",";
        file << missile_hist.vx[i] << "," << missile_hist.vy[i] << ",";
        file << missile_hist.ax_cmd[i] << "," << missile_hist.ay_cmd[i] << "," << missile_hist.hd[i] << ",";
        file << targ_hist.x[i] << "," << targ_hist.y[i] << ",";
        file << targ_hist.vx[i] << "," << targ_hist.vy[i] << "," << targ_hist.v[i] << ",";
        file << targ_hist.a[i] << "," << targ_hist.gam[i] << "," << targ_hist.gamd[i] << ",";
        file << rel_hist.x[i] << "," << rel_hist.y[i] << "," << rel_hist.r[i] << "," << rel_hist.vx[i] << "," << rel_hist.vy[i] << ",";
        file << rel_hist.xlam[i] << "," << rel_hist.xlamd[i] << "," << rel_hist.vc[i] << "\n";
    }

    file.close();
}

void SimulatePNav2d(Missile& missile, Target& targ) {

    RelativeState rel = computeRelative(missile, targ);
    
    double xlead = asin(targ.v * sin(targ.gam - rel.xlam) / missile.v);
    double theta = rel.xlam + xlead;

    missile.vx = missile.v * cos(theta + missile.hd);
    missile.vy = missile.v * sin(theta + missile.hd);

    double t = 0.0;
    double h = 0.0;

    const double N = 4;

    Logger log;
    log.log(t, missile, targ, rel);

    // Engagement Loop
    while (rel.vc > 0) {

        if (rel.r > 10) {
            h = 1e-3;
        } else {
            h = 1e-4;
        }

        // Relative geometry
        rel = computeRelative(missile, targ);

        // Guidance law
        missile.compute_guide(N, rel.vc, rel.xlamd, rel.xlam);

        // Dynamics
        missile.update(h);
        targ.update(h, t);

        t += h;

        log.log(t, missile, targ, rel);

        cout << "t: " << t << ", r: " << rel.r << endl;

    }

    log.exportCSV("sim.csv");
}

