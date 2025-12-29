#include "pnav.h"
#include <vector>
#include <iostream>

using namespace std;

int main() {

    Missile missile;
    Target targ;

    double m_x0 = 0;
    double m_y0 = 0;
    double m_v0 = 3000;
    double m_hd0 = 0; // deg
 
    missile.init(m_x0, m_y0, m_v0, m_hd0);

    double targ_x0 = 200000;
    double targ_y0 = 100000;
    double targ_v0 = 1000;
    double targ_a0 = 0;
    double targ_gam0 = 0; 
    targ.init(targ_x0, targ_y0, targ_v0, targ_a0, targ_gam0);

    SimulatePNav2d(missile, targ);

    return 0;
}