#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <acado_code_generation.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>


using namespace std;

USING_NAMESPACE_ACADO

int main( )
{
   
    // INTRODUCE THE VARIABLES (acadoVariables.x):
    // -------------------------
    //机器人的位置，偏航角
    ACADO::DifferentialState x;
    ACADO::DifferentialState y;
    ACADO::DifferentialState q;

    //Vx,Vy线速度 w角速度
    ACADO::Control vx;
    ACADO::Control vy;
    ACADO::Control w;


    double L = 0.3;         // vehicle wheel base
    double dt = 0.1;        // sampling time for discrete-time system

    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    ACADO::DifferentialEquation f;
    // model equations 全向轮运动模型
    f << dot(x) == vx*cos(q) - vy*sin(q);
    f << dot(y) == vx*sin(q) + vy*cos(q);
    f << dot(q) == w;

    //  
    // Weighting matrices and reference functions (acadoVariables.y)
    //
    // 代码中的 ACADO::Function 是 ACADO 库中的一个类 用于表示一个函数或一组函数。
    // 这些函数可以是标量、向量或矩阵值的，并且可以用于定义优化问题的目标函数或约束条件
    ACADO::Function rf;
    ACADO::Function rfN;

    rf  << x << y << q << vx << vy << w;
    rfN << x << y << q;

    const int N  = 30;
    const int Ni = 9;
    const double Ts = 0.1;

    // Provide defined weighting matrices:
    BMatrix W = eye<bool>(rf.getDim());
    BMatrix WN = eye<bool>(rfN.getDim());

    ACADO::OCP ocp(0, N * Ts, N);

    ocp.subjectTo( f );
    // control constraints
    ocp.subjectTo( -1.0 <= vx <= 1.0 );
    ocp.subjectTo( -1.0 <= vy <= 1.0 );
    ocp.subjectTo( -M_PI/4 <= w <= M_PI/4 );

    // obstacle contraints 加上障碍物约束
    // for(int i = 0; i < NUM_OBST; i++){
    //   ocp.subjectTo(sqrt((x[0]-obst[i][0])*(x[0]-obst[i][0]) + (x[2]-obst[i][1])*(x[2]-obst[i][1])) >= OBST_THRS);
    // }

    ocp.minimizeLSQ(W, rf);
    ocp.minimizeLSQEndTerm(WN, rfN);

    //设置非线性动态约束的数量：
    ocp.setNOD(obstacles.size() * 4);

    //
    // Export the code:
    // // 创建了一个 OCPexport 对象 mpc，它将用于导出OCP求解器的代码。
    // 设置MPC求解器的参数：
    OCPexport mpc( ocp );

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    //mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);        
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_RK45);
    // mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
    mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
    mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
    //	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);
    //	mpc.set(QP_SOLVER, QP_FORCES);
    mpc.set(MAX_NUM_QP_ITERATIONS, 999);
    mpc.set(HOTSTART_QP, YES);        
    //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);        
    mpc.set(LEVENBERG_MARQUARDT, 1.0E2);
    mpc.set(GENERATE_TEST_FILE, YES);
    mpc.set(GENERATE_MAKE_FILE, YES);
    mpc.set(GENERATE_MATLAB_INTERFACE, YES);
    //	mpc.set(USE_SINGLE_PRECISION, YES);
    mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
    mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);

    //	mpc.set(CG_USE_OPENMP, YES);
    // NOTE: This is crucial for export of MHE!
    //mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    //       mpc.set(FIX_INITIAL_STATE, YES);

    if (mpc.exportCode( "simple_mpc_export" ) != SUCCESSFUL_RETURN)
        exit( EXIT_FAILURE );

    mpc.printDimensionsQP( );

    return EXIT_SUCCESS;
}



