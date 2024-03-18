#include "mpc_controller_acado/mpc_controller_acado.h"

namespace rm_local_planner
{

mpc_controller_acado::mpc_controller_acado() {
    
   int N;
  int Ni;
  int max_obstacles; // should be the nearest ones
  double DT;
  double L_F;
  double L_R;
  double min_acc_dv;
  double max_acc_dv;
  double min_df_dv;
  double max_df_dv;
}







bool mpc_controller_acado::solve(Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state) {


 // INTRODUCE THE VARIABLES (acadoVariables.x):
  // -------------------------
  ACADO::DifferentialState x_dv;
  ACADO::DifferentialState y_dv;
  ACADO::DifferentialState psi_dv;
  ACADO::DifferentialState v_dv;

  ACADO::Control acc_dv;
  ACADO::Control df_dv;

  // DEFINE A DIFFERENTIAL EQUATION:
  // -------------------------------
  ACADO::DifferentialEquation f;

  // FULL ACKERMAN MODEL

  if (full_ackerman) {
    auto beta = atan(L_R / (L_R + L_F) * tan(df_dv));
    f << dot(x_dv) == v_dv * cos(psi_dv + beta);
    f << dot(y_dv) == v_dv * sin(psi_dv + beta);
    f << dot(psi_dv) == (v_dv / L_R * sin(beta));
    f << dot(v_dv) == acc_dv;
  } else {
    // Simlistic model for acceleration and angular speed control
    f << dot(x_dv) == v_dv * cos(psi_dv);
    f << dot(y_dv) == v_dv * sin(psi_dv);
    f << dot(psi_dv) == df_dv;  // and angular speed
    f << dot(v_dv) == acc_dv;   // control acceleration and
  }

  struct Obstacle // Defined by ellipses TODO(jediofgever), ellipses
  {//除了模型预测控制（MPC）中的状态变量、控制变量和参数之外的其他数据。
   //这些数据可能包括但不限于观测值、参考轨迹、传感器读数或其他与实时控制相关的信息
    ACADO::OnlineData h; // center x
    ACADO::OnlineData k; // center y
    ACADO::OnlineData a; // length along x axis
    ACADO::OnlineData b; // length along y axis
  };

  std::vector<Obstacle> obstacles;
  ACADO::DifferentialState obstacle_cost;

  //Costs related to obstacles
  for (size_t i = 0; i < max_obstacles; i++) {
    Obstacle obs;
    auto obs_expression =
      pow((x_dv - obs.h), 2) / pow((obs.a / 2.0) + robot_radius, 2) +
      pow((y_dv - obs.k), 2) / pow((obs.b / 2.0) + robot_radius, 2);

    obs_expression = exp(1.0 / obs_expression);
    obstacle_cost += obs_expression;
    obstacles.push_back(obs);
  }

  ACADO::Function rf;
  ACADO::Function rfN;

  rf << x_dv << y_dv << psi_dv << v_dv << acc_dv << df_dv << obstacle_cost;
  rfN << x_dv << y_dv << psi_dv << v_dv;

  ACADO::OCP ocp(0, N * DT, N);
  // dynamics
  ocp.subjectTo(f);  //方法添加动态约束。这里的 f 应该是一个表示系统动态的函数，例如车辆的运动方程。
  // control constraints with jerk  
  ocp.subjectTo(min_acc_dv <= acc_dv <= max_acc_dv);
  ocp.subjectTo(min_df_dv <= df_dv <= max_df_dv);


  // obstacle constraints
  for (auto && i : obstacles) {
    //ocp.subjectTo(
    //pow((x_dv - i.h), 2) / pow(i.a, 2) +
    //pow((y_dv - i.k), 2) / pow(i.b, 2) >= (robot_radius + 1.0));
  }

  // Provide defined weighting matrices:  定义权重矩阵 W 和 WN：
  ACADO::BMatrix W = ACADO::eye<bool>(rf.getDim());   //eye 函数创建了一个单位矩阵，其中的对角线元素为1，其余元素为0
  ACADO::BMatrix WN = ACADO::eye<bool>(rfN.getDim());

  //设置最小二乘问题：
  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);
//这两行代码设置了OCP的目标函数，即最小化状态和终端状态的加权二次误差。
//设置非线性动态约束的数量：
  ocp.setNOD(obstacles.size() * 4);

// 创建了一个 OCPexport 对象 mpc，它将用于导出OCP求解器的代码。
// 设置MPC求解器的参数：
  ACADO::OCPexport mpc(ocp);
  USING_NAMESPACE_ACADO

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);
  mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(GENERATE_TEST_FILE, YES);
  mpc.set(CG_USE_OPENMP, YES);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  mpc.set(FIX_INITIAL_STATE, YES);

}
}