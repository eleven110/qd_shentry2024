#pragma once

#include <vector>
#include "Eigen/Eigen"
#include <cppad/ipopt/solve.hpp>

using namespace std;

namespace rm_local_planner
{
class mpc_controller_cppad
{
private:
    
public:
    mpc_controller_cppad(/* args */);
    // MPC(const std::map<string, double> &params);

    ~mpc_controller_cppad();

    vector<double> solve(Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state);
    
private:

        // Parameters for mpc solver
        double          _max_angvel, 
                        _max_throttle, 
                        _bound_value;

        int             _mpc_steps, 
                        _x_start, 
                        _y_start, 
                        _theta_start, 
                        _v_start, 
                        _etheta_start, 
                        _angvel_start, 
                        _a_start;
        
        std::map<string, double> _params;
        
        int             n_vars_;
        int             n_constraints_;

        vector<double>  mpc_var_init_;

};


}
