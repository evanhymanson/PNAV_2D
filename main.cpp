#include "pnav.h"
#include <vector>
#include <iostream>

using namespace std;

int main() {

    int engagement_id = 0;
    double m_x0 = 0; // m
    double m_y0 = 0; // m

    vector<double> tx = {30000}; // {10000, 20000, 30000};              // 3
    vector<double> ty = {20000}; // {0, 10000, 20000};                  // 3
    vector<double> tv = {2000}; //{500, 1000, 1500, 2000};           // 4
    vector<double> ta = {5}; // {2, 5};                     // 2
    vector<double> tgam = {90}; // {-90, 0, 90};              // 3
    vector<double> mv = {3000}; // {2000, 2500, 3000};          // 3
    vector<double> mhd = {30}; // {-30, 0, 30};               // 3
    vector<double> tau = {.6}; // {.2, .6};                   // 2

    
    vector<Maneuver> maneuvers = {
        Maneuver::CONSTANT_TURN,
        // Maneuver::WEAVE,
        // Maneuver::BARREL_ROLL,
        // Maneuver::SPLIT_S,
        // Maneuver::SPIRAL_DIVE,
        // Maneuver::BANG_BANG
    };


    for (double targ_x0: tx) {
        for (double targ_y0: ty) {
            for (double targ_v0: tv) {
                for (double targ_a0: ta) {
                    for (double targ_gam0: tgam) {
                        for (double m_v0: mv) {
                            for (double m_hd0: mhd) {
                                for (double m_tau: tau) {
                                    for (Maneuver man: maneuvers) {
                                        
                                        cout << "Engagement_id: " << engagement_id << endl;

                                        Missile missile;
                                        Target targ;
                                        missile.init(m_x0, m_y0, m_v0, m_hd0, m_tau);
                                        targ.init(targ_x0, targ_y0, targ_v0, targ_a0, targ_gam0, man);

                                        string csvname = "training_data/engagement_" + to_string(engagement_id) + "_man" + to_string((int)man) + ".csv";
                                        SimulatePNav2d(missile, targ, csvname);

                                        engagement_id++;


                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    

    return 0;
}