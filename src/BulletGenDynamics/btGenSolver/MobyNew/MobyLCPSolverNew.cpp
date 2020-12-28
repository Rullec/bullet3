#include "MobyLCPSolverNew.h"
#include "unrevised_lemke_solver.h"

extern void GetLCP(const int size, tMatrixXd &A, tVectorXd &b);
extern void calc_min(double &left_min, double &perp_vio, const tMatrixXd &A,
                     const tVectorXd &b, const tVectorXd &sol);
extern bool judge_fail(double left_min, double perp_vio);
extern double calc_sol_dist(const tVectorXd &a, const tVectorXd &b);

btGenMobyLCPSovlerNew::btGenMobyLCPSovlerNew()
    : cLCPSolverBase(eLCPSolverType::MobyNew)
{
    mSolver = new UnrevisedLemkeSolver();
}
btGenMobyLCPSovlerNew::~btGenMobyLCPSovlerNew() { delete mSolver; }
#include <iostream>
int btGenMobyLCPSovlerNew::Solve(int num_of_vars, const tMatrixXd &A,
                                 const tVectorXd &b, tVectorXd &x)
{
    x.noalias() = tVectorXd::Zero(num_of_vars);
    double fast_left_min, fast_perp_vio;

    int pivots = -1;
    // mSolver->SolveLcpLemke(A, b, &x, &pivots, 1e-10);
    mSolver->SolveLcpLemke(A, b, &x, &pivots, 1e-10);
    calc_min(fast_left_min, fast_perp_vio, A, b, x);
    bool fast_fail = judge_fail(fast_left_min, fast_perp_vio);
    printf("[debug] moby1 fast left min %.5f, perp vio %.5f\n", fast_left_min,
           fast_perp_vio);
    if (fast_fail)
    {
        std::cout << "A = \n" << A << std::endl;
        std::cout << "b = \n" << b.transpose() << std::endl;
        exit(0);
    }
    return 0;
}

// void GetLCP(const int size, tMatrixXd &M, tVectorXd &q)
// {
//     M = tMatrixXd::Zero(size, size);
//     M.setRandom();
//     M = M.transpose() * M;
//     q = tVectorXd::Random(size);

//     // offical test case
//     // M.fill(1);
//     // q.fill(-1);
// }

// void calc_min(double &left_min, double &perp_vio, const tMatrixXd &A,
//               const tVectorXd &b, const tVectorXd &sol)
// {
//     printf("A size = %d %d\n", A.rows(), A.cols());
//     printf("b size = %d \n", b.size());
//     printf("sol size = %d \n", sol.size());
//     left_min = std::min(sol.minCoeff(), (A * sol + b).minCoeff());
//     perp_vio = std::fabs(sol.dot(A * sol + b));
// }
// bool judge_fail(double left_min, double perp_vio)
// {
//     return (left_min < -1e-6 || std::fabs(perp_vio) > 1e-6);
// }