#pragma once

#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include "tf2/utils.h"
#include <casadi/casadi.hpp>
#include <iostream>
#include <chrono>
#include <visualization_msgs/Marker.h>

using namespace casadi;

namespace rm_local_planner
{


class mpc_controller_casadi
{
public:
    mpc_controller_casadi();

    ~mpc_controller_casadi(){};

    Function setKinematicEquation();

    void setWeights(std::vector<double> weights);

    bool solve(Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state);

    std::vector<double> getFirstU();

    std::vector<double> getPredictX();

private:
    //mpc params
    int N_;  //horizon
    double dt_;  //step

    //状态与控制维度
    int dim_u = 3;
    int dim_x = 3;

    //constrains
    double ux_max_, ux_min_;
    double uy_min_, uy_max_;
    double w_max_, w_min_;
    
    //weights
    casadi::DM Q_, R_;            // 权重矩阵
    casadi::MX X, U;              // 系统状态和控制输入
  
    
    Function kinematic_equation_;


    std::unique_ptr<casadi::OptiSol> solution_;
};

}