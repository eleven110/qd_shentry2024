/**
 * *********************************************************
 *
 * @file: mpc_controller_osqp.cpp
 * @brief: Contains the mpc_controller_osqp class
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
#include "mpc_controller_osqp/mpc_controller_osqp.h"

namespace rm_local_planner
{
/**
 * @brief Construct a new LQR planner object
 */
mpc_controller_osqp::mpc_controller_osqp() 
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
    
    max_w_=4;
    max_v_=4;
    min_v_=0.0;
    p_ =12;
    m_ =8;

    max_v_inc_=1.57;

    max_vy_=4;
    min_vy_=0.0;
    
}


/**
 * @brief Execute MPC omni_control process 全向轮运动
 * @param robot_pose  robot current  state
 * @param traj        smooth traj  
 * @param base_odom   robot Odometry
 * @param cmd_vel     robot cmd_vel
 * @param du_p  previous control
 */
void mpc_controller_osqp::mpccomputeVelocityCommands(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel,const nav_msgs::Odometry& base_odom,Eigen::Vector3d &du_p_ ){
             ROS_INFO("mpc_controller_osqp  mpccomputeVelocityCommands ");                
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
            Eigen::Vector3d s_d(traj.poses[1].pose.position.x, traj.poses[1].pose.position.y, theta_trj); // desired state

            Eigen::Vector3d u_r(Vx, Vy, Vw);   // refered input
            // Eigen::Vector3d u = mpcControl(s, s_d, u_r, du_p_);
            Eigen::Vector2d diff_du_p (du_p_[0],du_p_[1]);

            Eigen::Vector2d u_r_diff(Vx, Vw); // refered input
            Eigen::Vector2d u = diff_mpcControl(s, s_d, u_r_diff,diff_du_p);
            printf("theta_trj: %f\n",theta_trj);
            printf("diff_distance: %f\n",diff_distance);

            // double u_v = linearRegularization(base_odom, u[0]);
            // double u_w = angularRegularization(base_odom, u[1]);
            // du_p_ = Eigen::Vector3d(u[0] - u_r[0], u[1] - u_r[1],regularizeAngle(u[1] - u_r[1]));

            diff_du_p = Eigen::Vector2d(u[0] - u_r[0], regularizeAngle(u[1] - u_r[1]));

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
 * @brief Execute MPC omni_control process 全向轮运动
 * @param s     current  state
 * @param s_d   desired  state
 * @param u_r   refered  control
 * @param du_p  previous control error
 * @return u    control  vector
 */
Eigen::Vector3d mpc_controller_osqp::mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector3d u_r,
                                        Eigen::Vector3d du_p)
{
    ROS_INFO("mpcControl mpcControl");                

    int dim_u = 3;
    int dim_x = 3;

    //   // state vector (5 x 1) 状态向量
    //   Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
    //   x.topLeftCorner(dim_x, 1) = s - s_d; //desired  state 期望状态：
    //   x[2] = regularizeAngle(x[2]); // x y  theta
    //   x.bottomLeftCorner(dim_u, 1) = du_p; //前一时刻的控制量  
    //   // original state matrix    初始状态矩阵
    //   Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
    //   A_o(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_; // v*sin() 参考速度*期望转向角度*控制频率
    //   A_o(1, 2) =  u_r[0] * cos(s_d[2]) * d_t_;
    //   // original control matrix  初始控制矩阵
    //   Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x, dim_u);
    //   B_o(0, 0) = cos(s_d[2]) * d_t_;
    //   B_o(1, 0) = sin(s_d[2]) * d_t_;
    //   B_o(2, 1) = d_t_;


    // state vector (5 x 1) 新的状态向量 [x y & Vx(k-1) Vy(k-1) W(k-1)]
    Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
    x.topLeftCorner(dim_x, 1) = s - s_d; //desired  state 期望状态：
    x[2] = x[2] - 2.0 * M_PI * std::floor((x[2] + M_PI) / (2.0 * M_PI));

    // x[2] = regularizeAngle(x[2]); // x y  theta
    x.bottomLeftCorner(dim_u, 1) = du_p; //前一时刻的控制量  

    //全向移动机器人。
    // state equation on error                        //离散线性化后的状态方程空间方程A
    Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();                         //[1 0 -(Vx*sin(&)+Vy*cos(&))*dt]
    A_o(0, 2) = -(u_r[0] * sin(s_d[2]) + u_r[1] * cos(s_d[2]))* d_t_;          //[0 1  (Vx*cos(&)-Vy*sin(&))*dt]
    A_o(1, 2) =  (u_r[0] * cos(s_d[2]) - u_r[1] * sin(s_d[2]))* d_t_;          //[0 0  1]

    Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x, dim_u); //[dt*cos(&) -dt*sin(&) 0]
    B_o(0, 0) =  cos(s_d[2]) * d_t_;//6x6                      //[dt*sin(&)  dt*cos(&) 0]
    B_o(0, 1) = -sin(s_d[2]) * d_t_;                           //[0 0 dt]
    B_o(1, 0) =  sin(s_d[2]) * d_t_; 
    B_o(1, 1) =  cos(s_d[2]) * d_t_;                         
    B_o(2, 2) =  d_t_;


    //构建新的状态空间A矩阵       [A_o B_o   ]
    //                           [0   I3x3  ]                     
    // state matrix (6x6)  state matrix  control matrix (6x6)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_x + dim_u);
    A.topLeftCorner(dim_x, dim_x)  = A_o;//(3,3)
    A.topRightCorner(dim_x, dim_u) = B_o;//(3,3)
    A.bottomLeftCorner(dim_u, dim_x) = Eigen::MatrixXd::Zero(dim_u, dim_x);//(3,3) 
    A.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix3d::Identity();//(3,3) 右下角矩阵的单位阵
    ROS_INFO("A_o B_o");                

    std::cout<<"A A"<<A<<std::endl;

    // control matrix (6 x 3)  新状态空间的B矩阵
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_u);
    B.topLeftCorner(dim_x, dim_u) = B_o;
    B.bottomLeftCorner(dim_u, dim_u) = Eigen::Matrix3d::Identity();
    ROS_INFO("B.topLeftCorner");                

    // output matrix(3 x 6) 控制的输出矩阵 [I3x3 0]
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x, dim_x + dim_u);
    C.topLeftCorner(dim_x, dim_x) = Eigen::Matrix3d::Identity();
    C.topRightCorner(dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);
    ROS_INFO("C.topLeftCorner");                

    // mpc state matrix(3p x 5)  // 你的矩阵A
    // predicting_time_domain: 12
    // control_time_domain: 8
    Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);  //nh.param("predicting_time_domain", p_, 4); nh.param("control_time_domain", m_, 4);
    //S_x为单独组成关于CA形式的大矩阵。
    Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * p_, dim_x + dim_u);//3x12,3+2  (36,5)
    for (int i = 0; i < p_; i++) //p_:12  
        S_x.middleRows(dim_x * i, dim_x) = C * A_pow(i + 1); //A_pow(i+1) 求矩阵A的（i+1）次幂。
    std::cout<<"S_x"<<S_x<<std::endl;
    ROS_INFO("A_pow A_pow");                

    
    // mpc control matrix(3p x 3m)  p_:14  m_:8  3*12  3*4 = (36,12)  控制的输出方程进行推导。
    Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_, dim_u * m_);//3x12,3x4 (36,12)
    for (int i = 0; i < p_; i++)  //p_:12
    {
        for (int j = 0; j < m_; j++)//m_:8
        {
        if (j <= i) //3 2 ,3行两列的块矩阵。
            S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C * A_pow(i - j) * B;
        else
            S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);
        //这个子矩阵的左上角位置是 ((dim_x * i), (dim_u * j))，大小为 dim_x 行和 dim_u 列
        //你可以在一个大矩阵中只填充你感兴趣的部分，而其他部分保持为零。
        //这是一种节省内存和计算资源的有效方法，特别是当处理大型稀疏矩阵时。
        }
    }

    // optimization  二次规划标准型
    // min 1/2 * x.T * H * x + g.T * x  计算代价函数的权重矩阵 H
    // s.t. l <= Px <= u   
    Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x * p_);                              // (3p x 1)
    //创建一个dim_x * p_大小的零向量Yr。这个向量可能代表了预测范围内的目标状态或者参考轨迹。    

    // 计算克罗内克积 其中 p_ 是预测范围，Q_ 是原始的代价矩阵。36*36
    Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_, p_), Q_);  // (3p x 3p) 
    std::cout<<"Q__"<<Q_<<std::endl;

    std::cout<<"Q"  <<Q <<std::endl;
    Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_, m_), R_);  // (2m x 2m)  // weight matrix for penalizing input error while tracking[v, w]
    //参数矩阵求解
    Eigen::MatrixXd H = S_u.transpose() * Q * S_u + R;                                   // (2m x 2m)
    Eigen::VectorXd g = S_u.transpose() * Q * (S_x * x - Yr);                            // (2m x 1)
    //S_u可能是控制增益矩阵，Q和R分别是状态和控制的代价矩阵。

    // boundary 控制量的边界值。
    Eigen::Vector3d u_min(min_v_,min_vy_, -max_w_);//min_v_:0 max_w_:1.57 弧度 1.57 弧度等于：度 = 1.57 × (180/π) ≈ 1.57 × 57.2958 ≈ 89.67 度
    Eigen::Vector3d u_max(max_v_,max_vy_,  max_w_); //max_v_:0.5 
    Eigen::Vector3d u_k_1(du_p[0], du_p[1], du_p[2]);//前一时刻的控制量
    Eigen::Vector3d du_min(-0.2, -0.2, -0.2);
    Eigen::Vector3d du_max(0.2,  0.2, M_PI);
    Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_min);    // (3m x 1)
    std::cout<<"U_min"<<U_min<<std::endl;
    Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_max);    // (3m x 1)
    std::cout<<"U_max"<<U_min<<std::endl;
    Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_r);        // (3m x 1)
    std::cout<<"U_r" <<U_r<<std::endl;
    Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_k_1);    // (3m x 1)
    Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_min);  // (3m x 1)  (24,1)
    std::cout<<"dU_min"<<dU_min<<std::endl;
    Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_max);  // (3m x 1)  (24,1)
    std::cout<<"dU_max"<<dU_max<<std::endl;

    // constriants
    Eigen::MatrixXd temp = Eigen::MatrixXd::Ones(m_, m_).triangularView<Eigen::Lower>(); //这是一个成员函数，它返回矩阵的下三角部分的视图。
    std::cout<<"temp"<<temp<<std::endl;
    Eigen::MatrixXd A_I = Eigen::kroneckerProduct(temp, Eigen::MatrixXd::Identity(dim_u, dim_u));  // (3m x 3m)

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2 * dim_u * m_, dim_u * m_);  // (4m x 2m)
    P.topRows(dim_u * m_) = A_I;
    P.bottomRows(dim_u * m_) = Eigen::MatrixXd::Identity(dim_u * m_, dim_u * m_);

    Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
    Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
    lower.topRows(dim_u * m_) = U_min - U_k_1 - U_r; //(24,1)
    lower.bottomRows(dim_u * m_) = dU_min;//(24,1)
    upper.topRows(dim_u * m_) = U_max - U_k_1 - U_r;//(24,1)
    upper.bottomRows(dim_u * m_) = dU_max;//(24,1)

    // solve
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(dim_u * m_);//3*8=24
    solver.data()->setNumberOfConstraints(2 * dim_u * m_);//2*3*8
    solver.data()->setHessianMatrix(_convertToSparseMatrix(H));//将会是一个稀疏矩阵，它包含了H中所有的非零元素，并且只占用了必要的存储空间
    solver.data()->setGradient(g);
    solver.data()->setLinearConstraintsMatrix(_convertToSparseMatrix(P));
    solver.data()->setLowerBound(lower);
    solver.data()->setUpperBound(upper);

    solver.initSolver();
    solver.solveProblem();

    Eigen::VectorXd solution = solver.getSolution();
    std::cout<<"solution"<<solution<<std::endl;//求出16个解

    // real control 本来就减去参考的控制差值输出的差值的控制量，后面输出控制应要加上差值减去的参考控制量
    // ur为参考控制量。
    Eigen::Vector3d u(solution[0] + du_p[0] + u_r[0],solution[1] + du_p[1] + u_r[1], regularizeAngle(solution[2] + du_p[2] + u_r[2]));

  return u;
}


/**
 * @brief Execute MPC diff_control process 差速轮运动
 * @param s     current  state
 * @param s_d   desired  state
 * @param u_r   refered  control
 * @param du_p  previous control error
 * @return u    control  vector
 */
Eigen::Vector2d mpc_controller_osqp::diff_mpcControl(Eigen::Vector3d s, Eigen::Vector3d s_d, Eigen::Vector2d u_r,
                                        Eigen::Vector2d diff_du_p)
{
  ROS_INFO("diff_mpcControl diff_mpcControl");                
  std::cout<<" diff_du_p "<<diff_du_p<<std::endl;
  int dim_u = 2;
  int dim_x = 3;

  // state vector (5 x 1) 状态向量
  Eigen::VectorXd x = Eigen::VectorXd(dim_x + dim_u);
  x.topLeftCorner(dim_x, 1) = s - s_d; //desired  state 期望状态：
  x[2] = regularizeAngle(x[2]); // x y  theta
  x.bottomLeftCorner(dim_u, 1) = diff_du_p; //前一时刻的控制量  

  // original state matrix    初始状态矩阵
  Eigen::Matrix3d A_o = Eigen::Matrix3d::Identity();
  A_o(0, 2) = -u_r[0] * sin(s_d[2]) * d_t_; // v*sin() 参考速度*期望转向角度*控制频率
  A_o(1, 2) =  u_r[0] * cos(s_d[2]) * d_t_;

  // original control matrix  初始控制矩阵
  Eigen::MatrixXd B_o = Eigen::MatrixXd::Zero(dim_x, dim_u);
  B_o(0, 0) = cos(s_d[2]) * d_t_;
  B_o(1, 0) = sin(s_d[2]) * d_t_;
  B_o(2, 1) = d_t_;

  //构建新的状态空间A矩阵       [A_o B_o   ]
  //                           [0   I3x3  ]                     
  // state matrix (5 x 5)  state matrix  control matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_x + dim_u);
  A.topLeftCorner(dim_x, dim_x)  = A_o;//(3,3)
  A.topRightCorner(dim_x, dim_u) = B_o;//(3,2)
  A.bottomLeftCorner(dim_u, dim_x) = Eigen::MatrixXd::Zero(dim_u, dim_x);//(2,3) 
  A.bottomRightCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();//(2,2) 右下角矩阵的单位阵
  std::cout<<"A A"<<A<<std::endl;

  // control matrix (5 x 2)  新状态空间的B矩阵
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dim_x + dim_u, dim_u);
  B.topLeftCorner(dim_x, dim_u) = B_o;
  B.bottomLeftCorner(dim_u, dim_u) = Eigen::Matrix2d::Identity();

  // output matrix(3 x 5) 控制的输出矩阵 [I3x3 0]
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_x, dim_x + dim_u);
  C.topLeftCorner(dim_x, dim_x) = Eigen::Matrix3d::Identity();
  C.topRightCorner(dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);

  // mpc state matrix(3p x 5)  // 你的矩阵A
  // predicting_time_domain: 12
  // control_time_domain: 8
  Eigen::MatrixPower<Eigen::MatrixXd> A_pow(A);  //nh.param("predicting_time_domain", p_, 4); nh.param("control_time_domain", m_, 4);
  //S_x为单独组成关于CA形式的大矩阵。
  Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(dim_x * p_, dim_x + dim_u);//3x12,3+2  (36,5)
  for (int i = 0; i < p_; i++) //p_:12  
    S_x.middleRows(dim_x * i, dim_x) = C * A_pow(i + 1); //A_pow(i+1) 求矩阵A的（i+1）次幂。
  std::cout<<"S_x"<<S_x<<std::endl;

 
  // mpc control matrix(3p x 2m)  p_:14  m_:8  3*4  2*4 = (12,8)  控制的输出方程进行推导。
  Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(dim_x * p_, dim_u * m_);//3x12,2x4 (36,8)
  for (int i = 0; i < p_; i++)  //p_:12
  {
    for (int j = 0; j < m_; j++)//m_:8
    {
      if (j <= i) //3 2 ,3行两列的块矩阵 出处方程U矩阵的参数矩阵
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = C * A_pow(i - j) * B;
      else
        S_u.block(dim_x * i, dim_u * j, dim_x, dim_u) = Eigen::MatrixXd::Zero(dim_x, dim_u);
      //这个子矩阵的左上角位置是 ((dim_x * i), (dim_u * j))，大小为 dim_x 行和 dim_u 列
      //你可以在一个大矩阵中只填充你感兴趣的部分，而其他部分保持为零。
      //这是一种节省内存和计算资源的有效方法，特别是当处理大型稀疏矩阵时。
    }
  }

  // optimization  二次规划标准型
  // min 1/2 * x.T * H * x + g.T * x  计算代价函数的权重矩阵 H
  // s.t. l <= Px <= u   
  Eigen::VectorXd Yr = Eigen::VectorXd::Zero(dim_x * p_);                              // (3p x 1)
  //创建一个dim_x * p_大小的零向量Yr。这个向量可能代表了预测范围内的目标状态或者参考轨迹。    

  // 计算克罗内克积 其中 p_ 是预测范围，Q_ 是原始的代价矩阵。36*36
  Eigen::MatrixXd Q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(p_, p_), Q_);  // (3p x 3p) 
  // std::cout<<"Q__"<<Q_<<std::endl;

  // std::cout<<"Q"  <<Q <<std::endl;
  Eigen::MatrixXd R = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_, m_), diff_R_);  // (2m x 2m)  // weight matrix for penalizing input error while tracking[v, w]
  //参数矩阵求解
  Eigen::MatrixXd H = S_u.transpose() * Q * S_u + R;                                   // (2m x 2m)
  Eigen::VectorXd g = S_u.transpose() * Q * (S_x * x - Yr);                            // (2m x 1)
  //S_u可能是控制增益矩阵，Q和R分别是状态和控制的代价矩阵。

  // boundary
  Eigen::Vector2d u_min(min_v_, -max_w_);//min_v_:0 max_w_:1.57 弧度 1.57 弧度等于：度 = 1.57 × (180/π) ≈ 1.57 × 57.2958 ≈ 89.67 度
  Eigen::Vector2d u_max(max_v_, max_w_); //max_v_:0.5 
  Eigen::Vector2d u_k_1(diff_du_p[0], diff_du_p[1]);//前一时刻的控制量
  Eigen::Vector2d du_min(-0.2, -0.2);
  Eigen::Vector2d du_max(0.2, M_PI);
  Eigen::VectorXd U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_min);    // (2m x 1)
  // std::cout<<"U_min"<<U_min<<std::endl;
  Eigen::VectorXd U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_max);    // (2m x 1)
  // std::cout<<"U_max"<<U_min<<std::endl;
  Eigen::VectorXd U_r = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_r);        // (2m x 1)
  // std::cout<<"U_r" <<U_r<<std::endl;
  Eigen::VectorXd U_k_1 = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), u_k_1);    // (2m x 1)
  Eigen::VectorXd dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_min);  // (2m x 1)  (16,1)
  // std::cout<<"dU_min"<<dU_min<<std::endl;
  Eigen::VectorXd dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(m_), du_max);  // (2m x 1)  (16,1)
  // std::cout<<"dU_max"<<dU_max<<std::endl;

  // constriants
  Eigen::MatrixXd temp = Eigen::MatrixXd::Ones(m_, m_).triangularView<Eigen::Lower>(); //这是一个成员函数，它返回矩阵的下三角部分的视图。
  // std::cout<<"temp"<<temp<<std::endl;
  Eigen::MatrixXd A_I = Eigen::kroneckerProduct(temp, Eigen::MatrixXd::Identity(dim_u, dim_u));  // (2m x 2m)

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2 * dim_u * m_, dim_u * m_);  // (4m x 2m)
  P.topRows(dim_u * m_) = A_I;
  P.bottomRows(dim_u * m_) = Eigen::MatrixXd::Identity(dim_u * m_, dim_u * m_);

  Eigen::VectorXd lower = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(2 * dim_u * m_);  // (4m x 1)
  lower.topRows(dim_u * m_) = U_min - U_k_1 - U_r;
  lower.bottomRows(dim_u * m_) = dU_min;
  upper.topRows(dim_u * m_) = U_max - U_k_1 - U_r;
  upper.bottomRows(dim_u * m_) = dU_max;

  // solve
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(dim_u * m_);//2*8=16
  solver.data()->setNumberOfConstraints(2 * dim_u * m_);//2*2*8
  solver.data()->setHessianMatrix(_convertToSparseMatrix(H));//将会是一个稀疏矩阵，它包含了H中所有的非零元素，并且只占用了必要的存储空间
  solver.data()->setGradient(g);
  solver.data()->setLinearConstraintsMatrix(_convertToSparseMatrix(P));
  solver.data()->setLowerBound(lower);
  solver.data()->setUpperBound(upper);

  solver.initSolver();
  solver.solveProblem();

  Eigen::VectorXd solution = solver.getSolution();
  std::cout<<"solution"<<solution<<std::endl;//求出16个解

  // real control 本来就减去参考的控制差值输出的差值的控制量，后面输出控制应要加上差值减去的参考控制量
  // ur为参考控制量。
  Eigen::Vector2d u(solution[0] + diff_du_p[0] + u_r[0], regularizeAngle(solution[1] + diff_du_p[1] + u_r[1]));

  return u;
}



/**
 * @brief convert matrix A to its sparse view
 * @param A     dense  matrix
 * @return A_s  sparse matrix
 */
Eigen::SparseMatrix<double> mpc_controller_osqp::_convertToSparseMatrix(Eigen::MatrixXd A)
{
  int row = A.rows();
  int col = A.cols();
  Eigen::SparseMatrix<double> A_s(row, col);

  for (int i = 0; i < row; i++)
    for (int j = 0; j < col; j++)
      A_s.insert(i, j) = A(i, j);

  return A_s;
}

}