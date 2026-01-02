#include <iostream>
#include <vector>
#include <fstream>
#include "logger.h"
#include "pnav.h"

using namespace std;

void Logger::log(double t, const Missile& missile, const Target& targ, const RelativeState& rel, const Kalman_Filter& filter) {
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

    // filter history 
    filter_hist.xlam.push_back(filter.get_xh_xlam());
    filter_hist.xlamd.push_back(filter.get_xh_xlamd());
}

void Logger::exportCSV(const string& filename) const {
    ofstream file(filename);

    if (!file.is_open()) {
        cerr << "ERROR: Could not open file: " << filename << endl;
        return;
    }

    file << "time,m_x,m_y,m_vx,m_vy,m_ax_cmd,m_ay_cmd,m_hd,";
    file << "t_x,t_y,t_vx,t_vy,t_v,t_a,t_gam,t_gamd,";
    file << "rel_x,rel_y,rel_r,rel_vx,rel_vy,rel_xlam,rel_xlamd,rel_vc,xh_xlam,xh_xlamd\n";

    for (int i = 0; i < time.size(); i++) {
        file << time[i] << ",";
        file << missile_hist.x[i] << "," << missile_hist.y[i] << ",";
        file << missile_hist.vx[i] << "," << missile_hist.vy[i] << ",";
        file << missile_hist.ax_cmd[i] << "," << missile_hist.ay_cmd[i] << "," << missile_hist.hd[i] << ",";
        file << targ_hist.x[i] << "," << targ_hist.y[i] << ",";
        file << targ_hist.vx[i] << "," << targ_hist.vy[i] << "," << targ_hist.v[i] << ",";
        file << targ_hist.a[i] << "," << targ_hist.gam[i] << "," << targ_hist.gamd[i] << ",";
        file << rel_hist.x[i] << "," << rel_hist.y[i] << "," << rel_hist.r[i] << "," << rel_hist.vx[i] << "," << rel_hist.vy[i] << ",";
        file << rel_hist.xlam[i] << "," << rel_hist.xlamd[i] << "," << rel_hist.vc[i] << ",";
        file << filter_hist.xlam[i] << "," << filter_hist.xlamd[i] << "\n";
    }

    file.close();
}