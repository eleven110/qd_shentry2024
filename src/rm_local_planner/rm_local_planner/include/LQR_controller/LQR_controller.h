/**
 * *********************************************************
 *
 * @file: lqr_planner.h
 * @brief: Contains the linear quadratic regulator (LQR) local planner class
 * @author: Yang Haodong
 * @date: 2024-01-12
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
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
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>
// #include "utility.h"


namespace  rm_local_planner
{

/**
 * @brief A class implementing a local planner using the LQR
 */
class LQR_controller 
{
public:
    LQR_controller(); 

    ~LQR_controller(){};
    /**
    * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
    * @param cmd_vel will be filled with the velocity command to be passed to the robot base
    * @return true if a valid trajectory was found, else false
    */
    void lqrcomputeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                            const nav_msgs::Path& traj,
                            geometry_msgs::Twist& cmd_vel,const nav_msgs::Odometry& base_odom);
    /**
    * @brief Execute LQR control process
    * @param s   current state
    * @param s_d desired state
    * @param u_r refered control
    * @return u  control vector
    */
    Eigen::Vector3d _lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector3d u_r);

    Eigen::Vector2d  diff_lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r);

    
private:
//   double Vx;           // refered control  
//   double Vy;           // refered control  
//   double Vw;           // refered control  

  double d_t_;         // control time interval
  Eigen::Matrix3d Q_;  // state error matrix
  Eigen::Matrix3d R_;  // control error matrix
  Eigen::Matrix2d diff_R_;
  int max_iter_;       // maximum iteration for ricatti solution
  double eps_iter_;    // iteration ending threshold

};

}  // namespace rm_local_planner
