#include "role_manager.h"
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;
using namespace std;


/**
 * @brief Compute role assignments
 * 
 * @param parameters 
 *        teammates[22], opponents[22], roles[12], ball[4], is_teammate_down[11], my_index, our_max_speed, their_max_speed
 * @return role_assignments[player index AKA unum-1] = (Role Number, Role pos. x, Role pos. y)
 * @return kick (short/long) targets for AP: [short_x, short_y, short_score, long_x, long_y, long_score, is_long_to_goal]
 * 
 *         Role ID:
 *         GK  0
 *         CB  1
 *         MM1 2
 *         MM2 3
 *         MM3 4
 *         MM4 5
 *         LS  6
 *         RS  7
 *         LF  8
 *         RF  9
 *         AP  10
 *         AP2 11
 *         AP3 12
 */
py::array_t<float> compute( py::array_t<float> parameters ){

    // ================================================= 1. Parse data
    
    py::buffer_info parameters_buf = parameters.request();
    float* parameters_ptr = (float*)parameters_buf.ptr;

    float teammates[22], opponents[22], roles[12], ball[4];
    bool is_teammate_down[11];
    float role_assignment[11][3]={{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0},{-1,0,0}};
    float kick_targets[7] = {0};

    int j=0;
    for(int i=0; i<22; i++){ teammates[i]=parameters_ptr[j++]; }
    for(int i=0; i<22; i++){ opponents[i]=parameters_ptr[j++]; }
    for(int i=0; i<12; i++){ roles[i]    =parameters_ptr[j++]; }
    for(int i=0; i< 4; i++){ ball[i]     =parameters_ptr[j++]; }
    for(int i=0; i<11; i++){ is_teammate_down[i]=parameters_ptr[j++]; }
    int my_index = int(parameters_ptr[j++]);
    float our_max_speed   = parameters_ptr[j++];
    float their_max_speed = parameters_ptr[j];

    // ================================================= 2. Compute path

    manage(teammates, opponents, roles, ball, is_teammate_down, my_index, role_assignment, kick_targets, our_max_speed, their_max_speed);
    
    // ================================================= 3. Prepare data to return
    
    py::array_t<float> retval = py::array_t<float>(40); //allocate
    py::buffer_info buff = retval.request();
    float *ptr = (float *) buff.ptr;

    for(int i=0; i<33; i++){
        ptr[i] = ((float*)role_assignment)[i];
    }
    for(int i=0; i<7; i++){
        ptr[i+33] = kick_targets[i];
    }

    return retval;
}






using namespace pybind11::literals; // to add informative argument names as -> "argname"_a

PYBIND11_MODULE(role_manager, m) {  // the python module name, m is the interface to create bindings
    m.doc() = "Compute role assignments"; // optional module docstring

    // optional arguments names
    m.def("compute", &compute, "Compute role assignments", "parameters"_a); 
}

