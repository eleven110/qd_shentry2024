#pragma once

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/JointState.h"

#include <tf2/utils.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <Eigen/Eigen>
#include <chrono>
#include <roborts_msgs/GameStatus.h>
#include <roborts_msgs/PidPlannerStatus.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_core/base_local_planner.h>

// #include "mpc_controller_cppad/mpc_controller_cppad.h"
#include "mpc_controller_casadi/mpc_controller_casadi.h"
#include "LQR_controller/LQR_controller.h"
#include "cubic_spline/cubic_spline_ros.h"
#include "mpc_controller_osqp/mpc_controller_osqp.h"
#include "local_planner.h"
#include "utility.h"


namespace rm_local_planner
{

class control_base: public nav_core::BaseLocalPlanner, local_planner::LocalPlanner
{

public:
    
    //下面为nav_core::BaseLocalPlanner必需要实现的纯虚函数。 
    control_base();

    /**
    * @brief Construct a new MPC planner object
    */
    control_base(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);


    virtual ~control_base() = default;

    /**
    * @brief Initialization of the local planner
    * @param name        the name to give this instance of the trajectory planner
    * @param tf          a pointer to a transform listener
    * @param costmap_ros the cost map to use for assigning costs to trajectories
    */
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    /**
   * @brief Set the plan that the controller is following
   * @param orig_global_plan the plan to pass to the controller
   * @return true if the plan was updated successfully, else false
   */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
    * @brief Check if the goal pose has been achieved
    * @return true if achieved, false otherwise
    */
    bool isGoalReached();

    /**
    * @brief Given the current position, orientation, and velocity of the robot, compute the velocity commands
    * @param cmd_vel will be filled with the velocity command to be passed to the robot base
    * @return true if a valid trajectory was found, else false
    */
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


private:
    void ImuCallback(const sensor_msgs::Imu &msg);

    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);

    void Game_StateCallback(const roborts_msgs::GameStatusPtr &msg );

    bool Follower_StateReq(roborts_msgs::PidPlannerStatus::Request& req, roborts_msgs::PidPlannerStatus::Response& resp);

    void pid_slove(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel);

    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    
    void Plan(const ros::TimerEvent& event);

    void casadi_mpc_slove(geometry_msgs::Twist &cmd_vel,Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
private:
    bool initialized_;     // initialized flag
    bool goal_reached_;    // goal reached flag
    tf2_ros::Buffer* tf_;  // transform buffer

    ros::Publisher  gimbal_yaw_position_cmd_;
    ros::Publisher  gimbal_pitch_position_cmd_;
    ros::Publisher  cmd_vel_pub_;
    ros::Publisher  local_path_pub_;
    ros::Publisher  predict_path_pub;
    ros::Subscriber imu_sub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber game_state_sub_;
    ros::Subscriber odom_sub_;
    ros::Timer plan_timer_;
    ros::ServiceServer planner_server_;

    std::shared_ptr<tf::TransformListener> tf_listener_;
    tf::StampedTransform global2path_transform_;
    nav_msgs::Odometry base_odom_;
    nav_msgs::Path global_path_;

    std::unique_ptr<mpc_controller_casadi> casadi_mpc_ptr;
    // std::unique_ptr<mpc_controller_cppad> cppad_mpc_ptr;
    std::unique_ptr<LQR_controller> LQR_controller_ptr;
    std::unique_ptr<mpc_controller_osqp> mpc_controller_osqp_ptr;
    Eigen::Vector3d du_p_;  // previous control error

    // odometry helper
    base_local_planner::OdometryHelperRos* odom_helper_;
    bool plan_ = false;
    int plan_freq_;      //规划频率
    int prune_index_ = 0;
    int path_callback = 0;
    int N_steps_ = 20;   //mpc的预测步长
    int mpc_speed_value_;//mpc速度乘于一个系数

    double d_t_;         // control time interval
    double max_x_speed_;
    double max_y_speed_;


    double set_yaw_speed_;
    // pid 
    double p_value_;
    double i_value_;
    double d_value_;

    double goal_dist_tolerance_;
    double prune_ahead_dist_;
    
  
    std::string global_frame_;
    std::string robot_frame_;
    std::string controller_type_;//选择局部规划器控制算法


    double yaw_;  //机器人航向角
    
    uint8_t game_state_ = 4;
    int planner_state_ = 2; //规划状态 0：静止  1：原地小陀螺  2：路径跟踪

    // goal parameters
    double goal_x_, goal_y_;
    Eigen::Vector3d goal_rpy_;

    costmap_2d::Costmap2DROS* costmap_ros_;  // costmap(ROS wrapper)
    std::vector<geometry_msgs::PoseStamped> global_plan_;
};


double ABS_limit(double value,double limit)
{
  if(value<limit && value>-limit)
  {
    return 0;
  }
  else
  {
    return value;
  }

}

}