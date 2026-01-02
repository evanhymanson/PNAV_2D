#include "pnav.h"
#include <vector>
#include <iostream>

using namespace std;

int main() {

    Missile missile;
    Target targ;

    double m_x0 = 0; // m
    double m_y0 = 0; // m
    double m_v0 = 3000; // m/s
    double m_hd0 = 0; // deg
 
    missile.init(m_x0, m_y0, m_v0, m_hd0);

    double targ_x0 = 40000; // m
    double targ_y0 = 100000; // m
    double targ_v0 = 1000; // m/s
    double targ_a0 = 5*9.81; 
    double targ_gam0 = -90; // deg

    targ.init(targ_x0, targ_y0, targ_v0, targ_a0, targ_gam0, Maneuver::BANG_BANG);

    SimulatePNav2d(missile, targ);

    return 0;
}