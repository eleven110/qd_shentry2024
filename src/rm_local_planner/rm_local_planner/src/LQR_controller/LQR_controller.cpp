/**
 * *********************************************************
 *
 * @file: omni_lyapunov_controller.cpp
 * @brief: Contains the linear quadratic regulator (LQR) local planner class
 * @author: lin yanhong
 * @date: 2024-02-20
 * @version: 1.0
 *
 * Copyright (c) 2024, lin yanhong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "LQR_controller/LQR_controller.h"

namespace rm_local_planner
{
/**
 * @brief Construct a new LQR planner object
 */
LQR_controller::LQR_controller() 
{

  // weight matrix for penalizing state error while tracking [x,y,theta_trj]
    std::vector<double>diag_vec = {10.0,10.0,1.0};
    for (size_t i = 0; i < diag_vec.size(); ++i)
      Q_(i, i) = diag_vec[i];

    // weight matrix for penalizing input error while tracking[v, w]
    for (size_t i = 0; i < diag_vec.size(); ++i)
      R_(i, i) = diag_vec[i];

    for (size_t i = 0; i < diag_vec.size()-1; ++i)
      diff_R_(i, i) = diag_vec[i];
    // Vx = 1.0;           // refered control  
    // Vy = 1.0;           // refered control  
    // Vw = 0.1;  
    eps_iter_ = 1e-4;
    max_iter_  = 200;
    int plan_freq_ = 20;
    d_t_ = 1.0/plan_freq_;
}



void LQR_controller::lqrcomputeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel,const nav_msgs::Odometry& base_odom ){
             ROS_INFO("LQR_controller  lqrcomputeVelocityCommands ");                
            double yaw_ = tf2::getYaw(robot_pose.pose.orientation);
          
            //double theta_trj = GetYawFromOrientation(traj.poses[0].pose.orientation)- GetYawFromOrientation(robot_pose.pose.orientation);
            double theta_trj = atan2((traj.poses[1].pose.position.y-robot_pose.pose.position.y ),( traj.poses[1].pose.position.x-robot_pose.pose.position.x));
            double diff_distance = hypot(robot_pose.pose.position.x-traj.poses[1].pose.position.x,
               robot_pose.pose.position.y-traj.poses[1].pose.position.y);

            // set it from -PI t
            if(theta_trj > M_PI){
                theta_trj -= 2*M_PI;
            } else if(theta_trj < -M_PI){
                theta_trj += 2*M_PI;
            }
            // calculate look-ahead distance 总的和速度。
            double Vx = base_odom.twist.twist.linear.x;
            double Vy = base_odom.twist.twist.linear.y;

            double Vw = base_odom.twist.twist.angular.z;
            Eigen::Vector3d s(robot_pose.pose.position.x, robot_pose.pose.position.y, yaw_);  // current state
            Eigen::Vector3d s_d(traj.poses[1].pose.position.x, traj.poses[1].pose.position.y, theta_trj);                // desired state

            Eigen::Vector3d u_r(Vx, Vy, Vw);   // refered input
            Eigen::Vector3d u = _lqrControl(s, s_d, u_r);
                      

            // Eigen::Vector2d u_r_diff(Vx, Vw); // refered input
            // Eigen::Vector2d u = diff_lqrControl(s, s_d, u_r_diff);

            printf("theta_trj: %f\n",theta_trj);
            printf("diff_distance: %f\n",diff_distance);
            
            //TODO: 应该去掉速度的nan值。
            if(isnan(u[0])||isnan(u[1])||isnan(diff_distance)){
                ROS_ERROR("cmd_vel eror ouccur ");
            }
            cmd_vel.linear.x = u[0];
            cmd_vel.linear.y = u[1];
            std::cout << "cmd_vel.linear.x" <<u[0]*10<< std::endl;                   
            std::cout << "cmd_vel.linear.y" <<u[1]*10<< std::endl;                   

            // cmd_vel.angular.z = set_yaw_speed_;

            if(isnan(diff_distance)){
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            }
          
}



/**
 * @brief Execute LQR control process
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered control
 * @return u  control vector
 */
Eigen::Vector3d LQR_controller::_lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector3d u_r)
{
  Eigen::Vector3d u;
  Eigen::Vector3d e(s - s_d);
   e[2] = e[2] - 2.0 * M_PI * std::floor((e[2] + M_PI) / (2.0 * M_PI));

  //全向移动机器人。
  // state equation on error                       //离散线性化后的状态方程空间方程A
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();                         //[1 0 -(Vx*cos(&)+Vysin(&))*dt]
  A(0, 2) = -(u_r[0] * sin(s_d[2]) + u_r[1] * cos(s_d[2]))* d_t_;          //[0 1  (Vx*cos(&)-Vysin(&))*dt]
  A(1, 2) =  (u_r[0] * cos(s_d[2]) - u_r[1] * sin(s_d[2]))* d_t_;                                    //[0 0  1]

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 3); //[dt*cos(&) -dt*sin(&) 0]
  B(0, 0) =  cos(s_d[2]) * d_t_;                    //[dt*sin(&)  dt*cos(&) 0]
  B(0, 1) = -sin(s_d[2]) * d_t_;                    //[0 0 dt]
  B(1, 0) = sin(s_d[2]) * d_t_; 
  B(1, 1) = cos(s_d[2]) * d_t_;                         
  B(2, 2) = d_t_;


  // discrete iteration Ricatti equation
  Eigen::Matrix3d P, P_;
  P = Q_;//Q_ 给定状态代价矩阵
  for (int i = 0; i < max_iter_; ++i)//max_iter_:100
  {
    Eigen::Matrix3d temp = R_ + B.transpose() * P * B;
    P_ = Q_ + A.transpose() * P * A - A.transpose() * P * B * temp.inverse() * B.transpose() * P * A; //P_是黎卡提方程的解
    if ((P - P_).array().abs().maxCoeff() < eps_iter_)
      break;
    P = P_;
  }
  
  // feedback
  Eigen::MatrixXd K = -(R_ + B.transpose() * P_ * B).inverse() * B.transpose() * P_ * A;

  u = u_r + K * e;

  return u;
}


Eigen::Vector2d LQR_controller::diff_lqrControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
{
  Eigen::Vector2d u;
  Eigen::Vector3d e(s - s_d);
  e[2] = e[2] - 2.0 * M_PI * std::floor((e[2] + M_PI) / (2.0 * M_PI));
  ROS_INFO("11111  ");                

  // state equation on error                       //离散线性化后的状态方程空间方程A
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity(); //[1 0 -v*sin(&)*dt]
  A(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_;          //[0 1 v*cos(&)*dt]
  A(1, 2) = u_r[0] * cos(s_d[2]) * d_t_;           //[0 0 1]
  ROS_INFO("Matrix3d A   ");                

  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2); //[dt*cos(&) 0]
  B(0, 0) = cos(s_d[2]) * d_t_;                    //[dt*sin(&) 0]
  B(1, 0) = sin(s_d[2]) * d_t_;                    //[0 dt]
  B(2, 1) = d_t_;
  ROS_INFO(":MatrixXd B  ");                

  // discrete iteration Ricatti equation
  Eigen::Matrix3d P, P_;
  P = Q_;//Q_ 给定状态代价矩阵
  for (int i = 0; i < max_iter_; ++i)//max_iter_:100
  {
    Eigen::Matrix2d temp = diff_R_ + B.transpose() * P * B;
    P_ = Q_ + A.transpose() * P * A - A.transpose() * P * B * temp.inverse() * B.transpose() * P * A; //P_是黎卡提方程的解
    if ((P - P_).array().abs().maxCoeff() < eps_iter_)
      break;
    P = P_;
  }
  
  // feedback
  Eigen::MatrixXd K = -(diff_R_ + B.transpose() * P_ * B).inverse() * B.transpose() * P_ * A;

  u = u_r + K * e;

  return u;
}












}