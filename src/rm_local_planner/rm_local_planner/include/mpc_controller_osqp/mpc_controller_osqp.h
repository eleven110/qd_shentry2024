/**
 * *********************************************************
 *
 * @file: mpc_controller_osqp.h
 * @brief: Contains the mpc local planner class
 * @author: lin Yanhong
 * @date: 2024-02-20
 * @version: 1.0
 *
 * Copyright (c) 2024, lin Yanhong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#pragma once

#include <geometry_msgs/PointStamped.h>
#include <tf2/utils.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <unsupported/Eigen/KroneckerProduct>
#include <unsupported/Eigen/MatrixFunctions>
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>

namespace  rm_local_planner
{

/**
 * @brief A class implementing a local planner using the mpc
 */
class mpc_controller_osqp 
{
public:
      mpc_controller_osqp(); 

      ~mpc_controller_osqp(){};
      /**
      * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
      * @param cmd_vel will be filled with the velocity command to be passed to the robot base
      * @return true if a valid trajectory was found, else false
      */
      void mpccomputeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                            const nav_msgs::Path& traj,
                            geometry_msgs::Twist& cmd_vel,const nav_msgs::Odometry& base_odom,Eigen::Vector3d &du_p_);
      /**
      * @brief Execute mpc control process
      * @param s   current state
      * @param s_d desired state
      * @param u_r refered control
      * @return u  control vector
      */
      Eigen::Vector3d mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector3d u_r,
                                            Eigen::Vector3d du_p);


      Eigen::Vector2d diff_mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r,
                                        Eigen::Vector2d du_p);

      double regularizeAngle(double angle)
      {
            return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
      }
      /**

      * @brief convert matrix A to its sparse view
      * @param A     dense matrix
      * @return A_s  sparse matrix
      */
      Eigen::SparseMatrix<double> _convertToSparseMatrix(Eigen::MatrixXd A);

private:
  //   double Vx;           // refered control  
  //   double Vy;           // refered control  
  //   double Vw;           // refered control  
    double max_v_, min_v_, max_v_inc_;  // linear velocity
    double max_w_, min_w_, max_w_inc_;  // angular velocity
    double max_vy_,min_vy_;
    double d_t_;         // control time interval
    Eigen::Matrix3d Q_;  // state error matrix
    Eigen::Matrix3d R_;  // control error matrix
    Eigen::Matrix2d diff_R_;
    Eigen::Vector3d du_p_;  // previous control error
    Eigen::Vector2d diff_du_p;
    int max_iter_;       // maximum iteration for ricatti solution
    double eps_iter_;    // iteration ending threshold
    int p_;
    int m_;
};

}  // namespace rm_local_planner
