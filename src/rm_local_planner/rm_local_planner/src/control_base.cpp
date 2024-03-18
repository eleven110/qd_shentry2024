#include "control_base.h"


#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rm_local_planner::control_base, nav_core::BaseLocalPlanner)

namespace rm_local_planner
{


control_base::control_base() : initialized_(false), goal_reached_(false), tf_(nullptr)  //, costmap_ros_(nullptr)
{
    ros::NodeHandle nh("~");;
    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
    nh.param<double>("max_y_speed", max_y_speed_, 1.0);
    nh.param<double>("set_yaw_speed", set_yaw_speed_, 0.0);
    nh.param<double>("p_value", p_value_, 0.5);
    nh.param<double>("i_value", i_value_, 1);
    nh.param<double>("d_value", d_value_, 1);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
    nh.param<int>("plan_frequency", plan_freq_, 10);
    nh.param<int>("N_steps", N_steps_, 20);//mpc预测步长
    nh.param<int>("mpc_speed_value", mpc_speed_value_, 5);//mpc预测步长

    nh.param<std::string>("controller_type", controller_type_, "mpc");

    nh.param<std::string>("global_frame", global_frame_, "map");

    nh.param<std::string>("robot_frame", robot_frame_,  "base_footprint");


    gimbal_yaw_position_cmd_   = nh.advertise<std_msgs::Float64>("/auto_car/yaw_steering_position_controller/command",10);
    gimbal_pitch_position_cmd_ = nh.advertise<std_msgs::Float64>("/auto_car/pitch_steering_position_controller/command",10);
    predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);//mpc求解后的预测点轨迹。
    local_path_pub_=   nh.advertise<nav_msgs::Path>("/path", 5);
    cmd_vel_pub_    =  nh.advertise<geometry_msgs::Twist>("/base_vel",10);
    global_path_sub_=  nh.subscribe("/move_base/GlobalPlanner/plan", 5, &control_base::GlobalPathCallback,this);
    game_state_sub_ =  nh.subscribe("/game_state",5,&control_base::Game_StateCallback,this);
    //订阅mid-360的imu数据
    //imu_sub_ = nh.subscribe("/imu/data", 1, &control_base::ImuCallback, this);

    planner_server_ =  nh.advertiseService("/pid_planner_status",&control_base::Follower_StateReq, this);

    tf_listener_ = std::make_shared<tf::TransformListener>();

    casadi_mpc_ptr.reset(new mpc_controller_casadi());

    LQR_controller_ptr.reset(new LQR_controller());

    mpc_controller_osqp_ptr.reset(new mpc_controller_osqp());

    plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&control_base::Plan,this);
    this->d_t_ = 1.0/plan_freq_;  // control time interval
}


control_base::control_base(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros): control_base()
{
    initialize(name, tf, costmap_ros);
}

/**
 * @brief Initialization of the local planner
 * @param name        the name to give this instance of the trajectory planner
 * @param tf          a pointer to a transform listener
 * @param costmap_ros the cost map to use for assigning costs to trajectories
 */
void control_base::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    if (!initialized_)
    {
        initialized_ = true;
        ros::NodeHandle nh("~");
        nh.param<double>("max_x_speed", max_x_speed_, 1.0);
        nh.param<double>("max_y_speed", max_y_speed_, 1.0);
        nh.param<double>("set_yaw_speed", set_yaw_speed_, 0.0);
        nh.param<double>("p_value", p_value_, 0.5);
        nh.param<double>("i_value", i_value_, 1);
        nh.param<double>("d_value", d_value_, 1);
        nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
        nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
        nh.param<int>("plan_frequency", plan_freq_, 10);
        nh.param<int>("N_steps", N_steps_, 20);

        nh.param<std::string>("global_frame", global_frame_, "map");
        
        nh.param<std::string>("robot_frame", robot_frame_,  "base_footprint");

        gimbal_yaw_position_cmd_   = nh.advertise<std_msgs::Float64>("/auto_car/yaw_steering_position_controller/command",10);
        gimbal_pitch_position_cmd_ = nh.advertise<std_msgs::Float64>("/auto_car/pitch_steering_position_controller/command",10);
        predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);//mpc求解后的预测点轨迹。
        local_path_pub_=   nh.advertise<nav_msgs::Path>("/path", 5);
        cmd_vel_pub_    =  nh.advertise<geometry_msgs::Twist>("/base_vel",10);
        global_path_sub_=  nh.subscribe("/move_base/GlobalPlanner/plan", 5, &control_base::GlobalPathCallback,this);
        game_state_sub_ =  nh.subscribe("/game_state",5,&control_base::Game_StateCallback,this);
        odom_sub_ =  nh.subscribe("/odom",5,&control_base::odomCallback,this);

        //订阅mid-360的imu数据
        //imu_sub_ = nh.subscribe("/imu/data", 1, &control_base::ImuCallback, this);

        planner_server_ =  nh.advertiseService("/pid_planner_status",&control_base::Follower_StateReq, this);

        tf_listener_ = std::make_shared<tf::TransformListener>();

        casadi_mpc_ptr.reset(new mpc_controller_casadi());

        plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&control_base::Plan,this);
        this->d_t_ = 1.0/plan_freq_;  // control time interval
    }
    else
    {
        ROS_WARN("rm_local_planner has already been initialized.");
    }
}


/**
 * @brief Set the plan that the controller is following
 * @param orig_global_plan the plan to pass to the controller
 * @return true if the plan was updated successfully, else false
 */
bool control_base::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  ROS_INFO("Got new plan");

  // set new plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // reset plan parameters
  if (goal_x_ != global_plan_.back().pose.position.x || goal_y_ != global_plan_.back().pose.position.y)
  {
    goal_x_ = global_plan_.back().pose.position.x;
    goal_y_ = global_plan_.back().pose.position.y;
    goal_rpy_ = getEulerAngles(global_plan_.back());
    goal_reached_ = false;
  }

  return true;
}


/**实现base_local_planner的纯虚函数
 * @brief Check if the goal pose has been achieved
 * @return true if achieved, false otherwise
 */ 
bool control_base::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR("MPC planner has not been initialized");
    return false;
  }

  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    return true;
  }
  return false;
}

void control_base::Plan(const ros::TimerEvent& event){

    if (game_state_ == 4 && planner_state_ == 2)
        {            
            if (plan_ ){
                // std::cout<<"path_callback times"<<path_callback<<std::endl;
                auto begin = std::chrono::steady_clock::now();
                auto start = ros::Time::now();

                // 1. Update the transform from global path frame to local planner frame
                UpdateTransform(tf_listener_, global_frame_,
                                global_path_.header.frame_id, global_path_.header.stamp,
                                global2path_transform_);//source_time needs decided

                // 2. Get current robot pose in global path frame
                geometry_msgs::PoseStamped robot_pose;  
                GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

                // 3. Check if robot has already arrived with given distance tolerance
                if (GetEuclideanDistance(robot_pose,global_path_.poses.back())<= goal_dist_tolerance_
                    || prune_index_ == global_path_.poses.size() - 1){
                    plan_ = false;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = set_yaw_speed_;
                    cmd_vel.linear.z = 1;   // bool success or not
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Planning Success!");
                    du_p_ = Eigen::Vector3d(0, 0,0);
                    return;
                }

                // odometry observation - getting robot velocities in robot frame
                // nav_msgs::Odometry base_odom;
                // odom_helper_->getOdom(base_odom);

                // // calculate look-ahead distance
                // double vt = std::hypot(base_odom.twist.twist.linear.x, base_odom.twist.twist.linear.y);
                // double wt = base_odom.twist.twist.angular.z;

                // 4. Get prune index from given global path
                FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);// TODO: double direct prune index is needed later!

                // 5. Generate the prune path and transform it into local planner frame
                nav_msgs::Path prune_path, local_path;//cubic spline local_path 

                local_path.header.frame_id = global_frame_;
                prune_path.header.frame_id = global_frame_;

                geometry_msgs::PoseStamped tmp_pose;
                tmp_pose.header.frame_id = global_frame_;

                TransformPose(global2path_transform_, robot_pose, tmp_pose);
                prune_path.poses.push_back(tmp_pose);
                
                int i = prune_index_;

                //从prune_index_开始，转换19个全局路径的坐标点到map坐标系。
                while (i < global_path_.poses.size() && i - prune_index_< 19 ){
                    
                    TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
                    prune_path.poses.push_back(tmp_pose);
                    i++;
                }

                //如prune_index_=12 ，则i的输出就是17
                //从prune_index_开始，转换global_path的全部路径
                // while (i < global_path_.poses.size() ){                   
                //     TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
                //     prune_path.poses.push_back(tmp_pose);
                //     i++;
                // }
                
                // local_path_pub_.publish(local_path);
                if(controller_type_=="pid"){
                    // 6. Generate the cubic spline trajectory from above prune path
                    GenTraj(prune_path, local_path);
                    // 7. Follow the trajectory and calculate the velocity
                    geometry_msgs::Twist cmd_vel; //求解出在k时刻的最优控制值。
                    pid_slove(robot_pose, local_path, cmd_vel);//对优化后的轨迹进行轨迹跟踪。
                    cmd_vel_pub_.publish(cmd_vel);          
                }
                std::cout<<"controller_type_ "<<controller_type_<<std::endl;

                if(controller_type_=="mpc"){
                    if(prune_path.poses.size()>this->N_steps_-1){
                        Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(prune_path.poses.size(), 3);
                        for (int i = 0; i < prune_path.poses.size(); ++i) { //N_steps_
                            
                            desired_state(i, 0) = prune_path.poses[i].pose.position.x;
                            desired_state(i, 1) = prune_path.poses[i].pose.position.y;
                            desired_state(i, 2) = tf::getYaw(prune_path.poses[i].pose.orientation);
                        }
                        
                        Eigen::VectorXd robot_state(3);
                        robot_state << robot_pose.pose.position.x, robot_pose.pose.position.y, tf::getYaw(robot_pose.pose.orientation);
                        geometry_msgs::Twist cmd_vel; //求解出在k时刻的最优控制值。
                        casadi_mpc_slove(cmd_vel, robot_state, desired_state);

                        // auto result = cppad_mpc_ptr->solve(robot_state, desired_state);
                        // cmd_vel.linear.x = result[0]*5;
                        // cmd_vel.linear.y = result[1]*5;
                        // std::cout<<"global_path_ .size() :"<<global_path_.poses.size()<<std::endl;
                        // std::cout<<"prune_path.size()    :"<<prune_path.poses.size()<<std::endl;

                        
                        // LQR_controller_ptr->lqrcomputeVelocityCommands(robot_pose, local_path, cmd_vel,base_odom_);
                        // osqp mpc 求解
                        // mpc_controller_osqp_ptr->mpccomputeVelocityCommands(robot_pose, local_path, cmd_vel, base_odom_,du_p_);             
                        cmd_vel_pub_.publish(cmd_vel);          
                        // osqp mpc
                        // nav_msgs::Path predict_path;
                        // predict_path.header.frame_id = "base_footprint";
                        // predict_path.header.stamp = ros::Time::now();
                        // geometry_msgs::PoseStamped pose_msg;
                        // for (int i = 2; i < result.size(); i += 2) {
                        //     pose_msg.pose.position.x = result[i];
                        //     pose_msg.pose.position.y = result[i + 1];
                        //     predict_path.poses.push_back(pose_msg);
                        // }
                        // predict_path_pub.publish(predict_path);
                        // predict_path.poses.clear();
                    }
                }
                
                auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
                ROS_INFO("Planning takes %f ms and passed %d/%d.",
                        plan_time.count()/1000.,
                        prune_index_,
                        static_cast<int>(global_path_.poses.size()));
            }
            else
            {
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = set_yaw_speed_;
                cmd_vel.linear.z = 0;   // bool success or not
                cmd_vel_pub_.publish(cmd_vel);
            }

        }
    
    else if(planner_state_ == 1)
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = set_yaw_speed_;
        cmd_vel.linear.z = 0;   // bool success or not
        cmd_vel_pub_.publish(cmd_vel);
    }

    else{
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.z = 0;   // bool success or not
        cmd_vel_pub_.publish(cmd_vel);
    }
}


bool control_base::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get robot position in global frame
  geometry_msgs::PoseStamped robot_pose_odom, robot_pose_map;
  costmap_ros_->getRobotPose(robot_pose_odom);
  transformPose(tf_, global_frame_, robot_pose_odom, robot_pose_map);

  // transform global plan to robot frame
  std::vector<geometry_msgs::PoseStamped> prune_plan = prune(robot_pose_map);
  
}

void control_base::FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist){

            double dist_threshold = 10;// threshold is 10 meters (basically never over 10m i suppose)
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist;
            if(prune_index!=0){
                sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index-1]);
            }else{
                sq_dist = 1e10;
            }

            double new_sq_dist = 0;
            while (prune_index < (int)path.poses.size()) {
                new_sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index]);
                if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {

                    //Judge if it is in the same direction and sq_dist is further than 0.3 meters
                    if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                        (path.poses[prune_index-1].pose.position.x - robot_pose.pose.position.x) +
                        (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                        (path.poses[prune_index-1].pose.position.y - robot_pose.pose.position.y) > 0
                        && sq_dist > prune_ahead_dist) {
                        prune_index--;
                    }else{
                        sq_dist = new_sq_dist;
                    }

                    break;
                }
                sq_dist = new_sq_dist;
                ++prune_index;
            }

            prune_index = std::min(prune_index, (int)(path.poses.size()-1));

}


/**
 * @brief Execute PID control process (with model)
 * @param s   current state
 * @param s_d desired state
 * @param u_r refered input
 * @return u  control vector
 */
// Eigen::Vector2d _pidControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r)
// {
//   Eigen::Vector2d u;
//   Eigen::Vector3d e(s_d - s);
//   Eigen::Vector2d sx_dot(k_ * e[0], k_ * e[1]);
//   Eigen::Matrix2d R_inv;
//   R_inv << cos(s[2]), sin(s[2]), -sin(s[2]) / i_value_, cos(s[2]) / i_value_;
//   u = R_inv * sx_dot;

//   return u;
// }

void control_base::casadi_mpc_slove(geometry_msgs::Twist &cmd_vel,Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state){
    Eigen::MatrixXd desired_state1 = desired_state.transpose();
    auto success = casadi_mpc_ptr->solve(robot_state, desired_state1);
    // cout << "solve success" << endl;                   
    auto control_cmd = casadi_mpc_ptr->getFirstU();
    // cout << "got cmd" << endl;
    // cout << "vx:"  <<  control_cmd[0] << endl;
    // cout << "vy:"  <<  control_cmd[1] << endl;
    cmd_vel.linear.x  = control_cmd[0]*mpc_speed_value_;
    cmd_vel.linear.y  = control_cmd[1]*mpc_speed_value_;
    cmd_vel.angular.z = set_yaw_speed_;
    
}


void control_base::pid_slove(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel){

            geometry_msgs::PoseStamped robot_pose_1;
            GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose_1); 

            yaw_ = tf::getYaw(robot_pose_1.pose.orientation);
            // std::cout<<"traj.poses[1]"<<traj.poses[1]<<std::endl;
            // std::cout<<"traj.poses[0]"<<traj.poses[0]<<std::endl;

            //double diff_yaw = GetYawFromOrientation(traj.poses[0].pose.orientation)- GetYawFromOrientation(robot_pose.pose.orientation);
            double diff_yaw = atan2((traj.poses[1].pose.position.y-robot_pose.pose.position.y ),( traj.poses[1].pose.position.x-robot_pose.pose.position.x));
            
            double diff_distance = GetEuclideanDistance(robot_pose,traj.poses[1]);

            // set it from -PI t
            if(diff_yaw > M_PI){
                diff_yaw -= 2*M_PI;
            } else if(diff_yaw < -M_PI){
                diff_yaw += 2*M_PI;
            }

            printf("diff_yaw: %f\n",diff_yaw);
            printf("diff_distance: %f\n",diff_distance);

            double vx_global = max_x_speed_*cos(diff_yaw)*p_value_;//*diff_distance*p_value_;
            double vy_global = max_y_speed_*sin(diff_yaw)*p_value_;//*diff_distance*p_value_;
            
            //TODO: 应该去掉速度的nan值。
            if(isnan(vx_global)||isnan(vy_global)||isnan(diff_distance)){
                ROS_ERROR("cmd_vel eror ouccur ");
            }
            cmd_vel.linear.x = vx_global * cos(yaw_) + vy_global * sin(yaw_);
            cmd_vel.linear.y = - vx_global * sin(yaw_) + vy_global * cos(yaw_);
            cmd_vel.angular.z = set_yaw_speed_;

            if(isnan(diff_distance)){
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            }
           

}


void control_base::GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
  if (!msg->poses.empty()){
      global_path_ = *msg;
      prune_index_ = 0;
      plan_ = true;
      path_callback++;
      std::cout<<"path_callback:::::::::::"<<path_callback<<std::endl;
      ROS_INFO("GlobalPathCallback GlobalPathCallback!");

  }
}

void control_base::Game_StateCallback(const roborts_msgs::GameStatusPtr &msg ){
    game_state_ = msg->game_state;
}


void control_base::ImuCallback(const sensor_msgs::Imu &msg)
{
  int a;
  //yaw_ = tf2::getYaw(msg.orientation);
  //ROS_INFO("imu_yaw:%f",yaw_);
}


void control_base::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO_ONCE("odom received!");

  //we assume that the odometry is published in the frame of the base
//   boost::mutex::scoped_lock lock(odom_mutex_);
  base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
  base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
  base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
  base_odom_.child_frame_id = msg->child_frame_id;
  //  ROS_DEBUG_NAMED("dwa_local_planner", "In the odometry callback with velocity values: (%.2f, %.2f, %.2f)",
  //      base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z);
}

// 服务处理函数
bool control_base::Follower_StateReq(roborts_msgs::PidPlannerStatus::Request& req,
          roborts_msgs::PidPlannerStatus::Response& resp){

    ROS_INFO("Request data : planner_state = %d, max_x_speed = %f, max_y_speed = %f, yaw_speed = %f"
            ,req.planner_state, req.max_x_speed, req.max_y_speed, req.yaw_speed);

    if (req.planner_state < 0 || req.planner_state > 2)
    {
        ROS_ERROR("Submitted data exception: Data cannot be negative");
        resp.result = 0;    //失败时返回0
        return false;
    }

    planner_state_ = req.planner_state;

    if(req.max_x_speed >0 && req.max_y_speed > 0)
    {
        max_x_speed_ = req.max_x_speed;
        max_y_speed_ = req.max_y_speed; 
    }
    if(req.yaw_speed > 0){
        set_yaw_speed_ = req.yaw_speed;
    }

    resp.result = 1;   //成功时返回1

    return true;
}


}


int main(int argc,char **argv)
{
  ros::init(argc, argv, "pid_position_follow");
  rm_local_planner::control_base robotctrl;
  ros::spin();
  return 0;
}
