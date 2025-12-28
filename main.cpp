#include "pnav.h"
#include <vector>
#include <iostream>

using namespace std;

int main() {

    Missile missile;
    Target targ;

    missile.init(0, 0, 3000, 50, 0, 0);

    targ.init(40000, 10000, 1000, 3, 0);

    SimulatePNav2d(missile, targ);

    return 0;
}