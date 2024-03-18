#include "mpc_controller_casadi/mpc_controller_casadi.h"

namespace rm_local_planner
{

mpc_controller_casadi::mpc_controller_casadi() {
    
    //输入维度
    dim_u = 3;
    dim_x = 3;
    N_ = 20;
    dt_ = 0.1;
    ux_max_ = 4;
    uy_max_ = 4;
    w_max_ = 2;
    std::vector<double> weights = {10, 10, 1, 10, 10, 0.4}; //Q,R # Q矩阵，size必须为 3, 分别表示状态误差 P.x, P.y, theta 的权重 
                                                      //# R矩阵，size必须为 3, 分别表示控制输入 vx, vy，w 的权重
    ux_min_ = - ux_max_;
    uy_min_ = - uy_max_;
    w_min_ = - w_max_;
    
        
    Q_ = casadi::DM::zeros(dim_u,dim_u);
    R_ = casadi::DM::zeros(dim_x,dim_x);
    
    

    setWeights(weights);
    kinematic_equation_ = setKinematicEquation();
}


// void local_obstacle_callback(const visualization_msgs::MarkerArray::SharedPtr msg)
//   {
//     obs_pos_list.clear();  // Clear the list before adding new positions

//     for (const auto & marker : msg->markers) {
//       if (marker.type == visualization_msgs::Marker::CYLINDER) {
//          casadi::DM pos = casadi::DM({marker.pose.position.x, marker.pose.position.y});
//          obs_pos_list.push_back(pos);
//       }
//     }

//     // Debug output
//     for (const auto & pos : obs_pos_list) {
//       RCLCPP_INFO(
//         this->get_logger(), "Obstacle position: (%.2f, %.2f)", pos(0).scalar(), pos(
//           1).scalar());
//     }

//   }


void mpc_controller_casadi::setWeights(std::vector<double> weights) {
    //cout << "setweights" << endl;
    Q_(0, 0) = weights[0];
    Q_(1, 1) = weights[1];
    Q_(2, 2) = weights[2];
    R_(0, 0) = weights[3];
    R_(1, 1) = weights[4];
    R_(2, 2) = weights[5];
    //cout << "set weight finish" << endl;

}

Function mpc_controller_casadi::setKinematicEquation() {
    //cout << "set kinematic" << endl;
    casadi::MX x = MX::sym("x");// MPC 状态包括(x, y, theta)，x位置，y位置和theta航向角
    casadi::MX y = MX::sym("y");
    casadi::MX theta = MX::sym("theta");
    casadi::MX state_vars = MX::vertcat({x, y, theta});

    casadi::MX vx = casadi::MX::sym("vx");   // MPC 控制输入是(vx, vy, w)，x速度,y速度和角速度
    casadi::MX vy = casadi::MX::sym("vy");   // MPC 控制输入是(vx, vy, w)，x速度,y速度和角速度
    casadi::MX w =  casadi::MX::sym("w");
    casadi::MX control_vars = casadi::MX::vertcat({vx, vy, w});
    
    // 全向运动学模型
    casadi::MX rhs = casadi::MX::vertcat({  vx * casadi::MX::cos(theta) - vy * casadi::MX::sin(theta),
                                            vx * casadi::MX::sin(theta) + vy * casadi::MX::cos(theta),
                                            w });

    return casadi::Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}


bool mpc_controller_casadi::solve(Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state) {
    
    std::cout << "mpc_controller_casadi::solve " << std::endl;
    casadi::Opti opti = casadi::Opti();
    std::cout << "casadi::Opti() " << std::endl;

    casadi::Slice all;

    casadi::MX cost = 0;

    this->X = opti.variable(3, N_);//x y threa
    this->U = opti.variable(3, N_-1);//控制量 vx vy w
    casadi::MX x =     X(0, all);
    casadi::MX y =     X(1, all);
    casadi::MX theta = X(2, all);
    casadi::MX vx =    U(0, all);
    casadi::MX vy =    U(1, all);
    casadi::MX w  =    U(2, all);


    casadi::MX X_ref = opti.parameter(3, N_);
    casadi::MX X_cur = opti.parameter(3);
    casadi::MX U_ref = opti.parameter(3,N_);//控制输出参考量

    casadi::DM x_tmp1 = {robot_state[0], robot_state[1], robot_state[2]};

    opti.set_value(X_cur, x_tmp1);  //set current state
    // cout << "set current state success" << endl;

    
    //按列索引 x y w 
    std::vector<double> X_ref_v(desired_state.data(), desired_state.data() + desired_state.size());
    //auto tp1 = std::chrono::steady_clock::now();

    casadi::DM X_ref_d(X_ref_v);
    
    //X_ref_d.resize(3, N_);
    
    //cout << "desired_state:" << desired_states << endl;
    //cout << "X_ref_v:" << X_ref_v << endl;
    //cout << "X_ref_d:" << X_ref_d << endl;
    // DM x_tmp2(3, N_ );
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j <= N_; ++j) {
    //         x_tmp2(i, j) = desired_states(i, j);
    //     }
    // }

    X_ref = MX::reshape(X_ref_d, 3, N_);
    //opti.set_value(X_ref, X_ref_d);  //set reference traj


    //设置一个控制的参考速度 好像并不能用。
    Eigen::MatrixXd desired_v = Eigen::MatrixXd::Zero(N_-1, 3);

    for (int i = 0; i < N_-1; ++i) { //N_steps_                        
        desired_v(i, 0) = 1;
        desired_v(i, 1) = 1;
        desired_v(i, 2) = 0;
    }
    Eigen::MatrixXd desired_v1 = desired_v.transpose();
    std::vector<double> U_ref_v(desired_v1.data(), desired_v1.data() + desired_v1.size());

   
    casadi::DM U_ref_d(U_ref_v);

    U_ref = MX::reshape(U_ref_d,3, N_-1);


    //set costfunction
    for (int i = 0; i < N_-1; ++i) {
        casadi::MX X_err = X(all, i) - X_ref(all, i); 
        casadi::MX U_0 = U(all, i);
        //cout << "U_0 size:" << U_0.size() << endl;
        //cout << "cost size:" << cost_.size() << endl;
        cost += MX::mtimes({X_err.T(), Q_, X_err});//衡量状态偏差
        //cout << "cost size:" << cost_.size() << endl; 
        cost += MX::mtimes({U_0.T(), R_, U_0});//衡量输入大小
        //cout << "cost size:" << cost_.size() << endl;
        //MX::mtimes 只适用于 MX 类型的对象  用于执行矩阵乘法
    }
    
    //cout << "cost size:" << cost_.size() << endl; xTQx  最后一个数据的损失函数。 
    //衡量最终状态偏差
    cost += MX::mtimes({(X(all, N_-1) - X_ref(all, N_-1)).T(), Q_,
                        X(all, N_-1) - X_ref(all, N_-1)});
                        
    //cout << "cost:" << cost << endl;
    opti.minimize(cost);
    //cout << "set cost success" << endl;

    //kinematic constrains  运动学约束
    for (int i = 0; i < N_-1; ++i) {
        std::vector<casadi::MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        casadi::MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);
        opti.subject_to(X_next == X(all, i + 1));
    }

    //init value
    opti.subject_to(X(all, 0) == X_cur);

    //speed angle_speed limit
    

    // 最大x方向速度，最大y方向速度和最大角速度约束
    opti.subject_to(ux_min_ <= vx <= ux_max_);
    opti.subject_to(uy_min_ <= vy <= uy_max_);
    opti.subject_to(w_min_  <= w  <= w_max_);

    //set solver
    casadi::Dict solver_opts;
    solver_opts["expand"] = true; //MX change to SX for speed up
    solver_opts["ipopt.max_iter"] = 100;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt.acceptable_tol"] = 1e-6;
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

    opti.solver("ipopt", solver_opts);

    //auto sol = opti.solve();
    solution_ = std::make_unique<casadi::OptiSol>(opti.solve());

    return true;
}

std::vector<double> mpc_controller_casadi::getFirstU() {
    std::vector<double> res;
    auto first_vx =  solution_->value(U)(0, 0);
    auto first_vy =  solution_->value(U)(1, 0);
    
    //cout << "first_u" << first_u << " " << "first_v" << first_v << endl;

    res.push_back(static_cast<double>(first_vx));
    res.push_back(static_cast<double>(first_vy));
    return res;
}

std::vector<double> mpc_controller_casadi::getPredictX() {
    std::vector<double> res;
    auto predict_x = solution_->value(X);
    //cout << "predict_x size :" << predict_x.size() << endl;
    for (int i = 0; i <= N_-1; ++i) {
        res.push_back(static_cast<double>(predict_x(0, i)));
        res.push_back(static_cast<double>(predict_x(1, i)));
    }
    return res;
}

}