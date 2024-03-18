#pragma once


#include <Eigen/Eigen>
#include <Eigen/Dense>



namespace rm_local_planner
{
class mpc_controller_casadi
{
public:
        mpc_controller_acado();

        ~mpc_controller_acado(){};

        bool solve(Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state);
private:

};

}