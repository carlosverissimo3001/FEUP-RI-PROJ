#include "role_manager.h"
#include <chrono>
#include <iostream>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

std::chrono::_V2::system_clock::time_point t1,t2;

float params[] = {
   -1.44580965e+01, -2.79213518e-01, -2.39663363e+00, -2.70384002e+00,
       -4.20992851e+00,  6.54071048e-02,  6.40676975e+00, -8.26445580e+00,
        5.86933517e+00, -9.50000000e+00,  1.19864929e+00, -4.55883360e+00,
        1.12767401e+01, -7.86146975e+00, -2.43103695e+00,  2.79730535e+00,
       -1.72000003e+00, -5.82000017e+00,  7.19470978e+00, -9.70270157e+00,
        8.17522049e+00, -5.33548975e+00, -3.01103187e+00,  7.26745576e-02,
       -1.03448546e+00,  3.10811710e+00, -9.96259630e-01, -3.00426674e+00,
        2.98169565e+00,  4.94772863e+00,  3.99908400e+00, -1.73028950e-02,
        2.99849916e+00, -5.06537056e+00,  7.96660948e+00, -8.83889294e+00,
        1.00000000e+03,  1.00000000e+03,  1.00000000e+03,  1.00000000e+03,
        1.00000000e+03,  1.00000000e+03,  1.00000000e+03,  1.00000000e+03,
       -1.44580965e+01, -2.79213518e-01, -1.72000003e+00, -5.82000017e+00,
        6.40676975e+00, -8.26445580e+00,  5.86933517e+00, -9.50000000e+00,
        8.17522049e+00, -5.33548975e+00,  1.12767401e+01, -7.86146975e+00,
        7.19999981e+00, -9.69999981e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  9.00000000e+00,
        5.00000000e-01,  5.00000000e-01
};
int params_size = sizeof(params)/sizeof(params[0]);


int main(){
    float teammates[22], opponents[22], roles[12], ball[4];
    bool is_teammate_down[11];
    float role_assignment[11][3]={{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0}};
    float kick_targets[7] = {0};

    int j=0;
    for(int i=0; i<22; i++){ teammates[i]=params[j++]; }
    for(int i=0; i<22; i++){ opponents[i]=params[j++]; }
    for(int i=0; i<12; i++){ roles[i]    =params[j++]; }
    for(int i=0; i< 4; i++){ ball[i]     =params[j++]; }
    for(int i=0; i<11; i++){ is_teammate_down[i]=params[j++]; }
    int my_index = int(params[j++]);
    float our_max_speed   = params[j++];
    float their_max_speed = params[j];


    // ================================================= 2. Compute path

    t1 = high_resolution_clock::now();
    manage(teammates, opponents, roles, ball, is_teammate_down, my_index, role_assignment, kick_targets, our_max_speed, their_max_speed);
    t2 = high_resolution_clock::now();
    

    std::cout << duration_cast<microseconds>(t2 - t1).count() << "us (includes initialization)\n";

    float role_assignment2[11][3]={{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0}};
    t1 = high_resolution_clock::now();
    manage(teammates, opponents, roles, ball, is_teammate_down, my_index, role_assignment2, kick_targets, our_max_speed, their_max_speed);
    t2 = high_resolution_clock::now();

    std::cout << duration_cast<microseconds>(t2 - t1).count() << "us\n";

}
