#include "ekf.hpp"

ekf::ekf(VectorXd x0, uint num_measurements)
{
    // Constructor
    this->num_states = x0.size();
    this->num_measurements = num_measurements;
    
}