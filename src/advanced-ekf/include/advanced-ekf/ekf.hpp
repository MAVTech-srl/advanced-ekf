#pragma once
#include <eigen3/Eigen/Core>

using namespace std;
using namespace Eigen;

class ekf
{
private:
    uint num_states;
    uint num_measurements;
public:
    ekf(VectorXd x0, uint num_measurements);
    ~ekf()
    {
        // Nothing
    };
    void transition_model(VectorXd new_state, const double dt, const VectorXd state);
    void measurement_model(VectorXd out, const VectorXd state);
    VectorXd get_x(void);
    MatrixXd get_P(void);
    MatrixXd Q, R;
    MatrixXd F, H;  // Jacobians
    VectorXd x0;    // Initial state mean
    MatrixXd P0;    // Initial state covariance
};


