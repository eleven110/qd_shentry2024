#include "mpc_controller_cppad/mpc_controller_cppad.h"


using CppAD::AD;
namespace rm_local_planner
{
    
// Ipopt：可以通过使得最小化成本函数，获得最优驱动输入[δ1,a1,…,δN−1,aN−1]，
// 由于需要加上驱动限制Ipopt 需要使用雅可比(jacobians)和黑塞(haishens)矩阵，他不会直接计算，所以使用到了CppAD。
const int N = 20;//预测时域（Prediction Horizon）：
const int VREF = 2;//期望速度。
const double dt = 0.1;

// 所有预测的状态值存储到一个向量里边，所以需要确定每种状态在向量中的开始位置
int x_start = 0;
int y_start   = x_start + N;//30
int psi_start = y_start + N;//60
int u_start = psi_start + N;//90

int v_start = u_start + N - 1;//119
int r_start = v_start + N - 1;//148
double num = 0;

mpc_controller_cppad::mpc_controller_cppad() {

    
}

mpc_controller_cppad::~mpc_controller_cppad() {}
class FG_eval
{
public:
    Eigen::MatrixXd desired_state_;

    FG_eval(Eigen::MatrixXd desired_state) {
        desired_state_ = desired_state;
    }
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

	// 该函数的目的是定义约束，fg向量包含的是总损失和约束，vars向量包含的是状态值和驱动器的输入
    void operator()(ADvector &fg, const ADvector &vars) {
        // 任何cost都会加到fg[0]
        //cost
        fg[0] = 0;
        //其中f[0] 表示代价函数，f[1]~f[6N+1] 表示约束函数
        //weights
        const int x_weight = 10;
        const int y_weight = 10;
        const int psi_weight =0;
        const int u_weight = 1;
        const int v_weight = 1;

		// 使车辆轨迹和参考路径的误差最小，且使车辆的速度尽量接近参考速度
        for (int i = 0; i < N; ++i) {
            // t+1时刻的状态
            AD<double> x_des = desired_state_(i, 0);
            AD<double> y_des = desired_state_(i, 1);
            AD<double> psi_des = desired_state_(i, 2);
            fg[0] += x_weight * CppAD::pow((vars[x_start + i] - x_des), 2);
            fg[0] += y_weight * CppAD::pow((vars[y_start + i] - y_des), 2);
            if (psi_des > M_PI) psi_des -= 2 * M_PI;
            if (psi_des < -M_PI) psi_des += 2 * M_PI;
            fg[0] += psi_weight * CppAD::pow(vars[psi_start + i] - psi_des + M_PI / 2, 2);
            //TODO:加上期望的Vx Vy速度做代价
            // fg[0] += 100 * CppAD::pow(VREF - vars[u_start + i], 2);
            // fg[0] += 100 * CppAD::pow(VREF - vars[v_start + i], 2);
            std::cout<<"const i :"  <<i<<std::endl;

        }

        //控制的代价值
        for (int i = 0; i < N-1; ++i) {
            fg[0] += u_weight * CppAD::pow(vars[u_start + i ] , 2);
            fg[0] += v_weight * CppAD::pow(vars[v_start + i ] , 2);
        }

		// 设置fg的初始值为状态的初始值，这个地方为初始条件约束
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        
        for (int i = 1; i < N; ++i) {
            //T+1时刻
            AD<double> x_1 = vars[x_start + i];
            AD<double> y_1 = vars[y_start + i];
            AD<double> psi_1 = vars[psi_start + i];

            //T时刻
            AD<double> x_0 = vars[x_start + i - 1];
            AD<double> y_0 = vars[y_start + i - 1];
            AD<double> psi_0 = vars[psi_start + i - 1];

            AD<double> u_0 = vars[u_start + i - 1];
            AD<double> v_0 = vars[v_start + i - 1];
            AD<double> r_0 = vars[r_start + i - 1];

            //u_0为V_x方向的速度，V_0为小车V_y方向上的速度。  x_1为广义坐标系x上的速度 ，y_1为广义坐标系y上的速度
            fg[1 + x_start + i] = x_1 - (x_0 + (u_0 * CppAD::cos(psi_0) - v_0 * CppAD::sin(psi_0)) * dt);
            fg[1 + y_start + i] = y_1 - (y_0 + (u_0 * CppAD::sin(psi_0) + v_0 * CppAD::cos(psi_0)) * dt);
            fg[1 + psi_start + i] = psi_1 - (psi_0 + r_0 * dt);
        }
        
    }
     
};

vector<double> mpc_controller_cppad::solve( Eigen::VectorXd robot_state,const Eigen::MatrixXd &desired_state) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    Eigen::VectorXd robot_state_;
    robot_state_ = robot_state;
    //赋值机器人状态量。
    const double x   = robot_state_[0]; 
    const double y   = robot_state_[1];
    const double psi = robot_state_[2];

    // 控制器的输入只有x ,y方向上的速度值。
	// 独立状态的个数，注意：驱动器的输入（N - 1）* 2  30*3+29*3 预测三个状态量，
    int n_var = N * 3 + (N - 1) * 3;   //每一个状态都对应一个控制量。 177

    // 约束条件的个数    
    int n_constraints = N * 3;

	// 除了初始值，初始化每一个状态为0
    Dvector vars(n_var);
    for (int i = 0; i < n_var; ++i) {
        vars[i] = 0.0;
    }

    //  设置每一个状态变量的最大和最小值
	// 【1】设置非驱动输入的最大和最小值
	// 【2】设置方向盘转动角度范围-25—25度
	// 【3】加速度的范围-1—1
    Dvector vars_lowerbound(n_var);
    Dvector vars_upperbound(n_var);

    //0-90
    for (int i = 0; i < u_start; ++i) {
        vars_lowerbound[i] = -1e19;
        vars_upperbound[i] = 1e19;
    }

    for (int i = u_start; i < v_start; ++i) {
        vars_lowerbound[i] = -3.2;
        vars_upperbound[i] = 3.2;
    }
    for (int i = v_start; i < r_start; ++i) {
        vars_lowerbound[i] = -3.2;
        vars_upperbound[i] = 3.2;        
    }
    for (int i = r_start; i < n_var; ++i) {
        vars_lowerbound[i] = -1;
        vars_upperbound[i] = 1;
    }

	// 设置约束条件的的最大和最小值，除了初始状态其他约束都为0
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;

    FG_eval fg_eval(desired_state);

    string options;
    options += "Integer print_level 0\n";
    options += "Sparse true forward\n";
    options += "Sparse true reverse\n";
    options += "Numeric max_cpu_time 0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval> (
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution
    );

    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    auto cost = solution.obj_value;//计算出来的代价值
    //cout << "cost:" << cost << " ";
    
	// 返回预测出的轨迹点状态
    vector<double> solved;
    solved.push_back(solution.x[u_start]);
    solved.push_back(solution.x[v_start]);
    for (int i = 0; i < N; ++i) {
        solved.push_back(solution.x[x_start + i]);
        solved.push_back(solution.x[y_start + i]);
    }
    //计算在K时刻的控制量。
    std::cout << "u:" << solution.x[u_start] << " " << "v:" << solution.x[v_start] << std::endl;
    return solved;
}

}