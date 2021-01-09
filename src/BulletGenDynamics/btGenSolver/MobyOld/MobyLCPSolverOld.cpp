#include "MobyLCPSolverOld.h"
#include "moby_lcp_internal.h"

void GetLCP(const int size, tMatrixXd &A, tVectorXd &b);
void calc_min(double &left_min, double &perp_vio, const tMatrixXd &A,
              const tVectorXd &b, const tVectorXd &sol);
bool judge_fail(double left_min, double perp_vio);
double calc_sol_dist(const tVectorXd &a, const tVectorXd &b)
{
    return (a - b).norm();
}
btGenMobyLCPSovler::btGenMobyLCPSovler()
    : cLCPSolverBase(eLCPSolverType::MobyOld)
{
    mOldX.resize(0);
    mSolver = new moby_lcp_solver();
}
btGenMobyLCPSovler::~btGenMobyLCPSovler() { delete mSolver; }
#include <iostream>
int btGenMobyLCPSovler::Solve(int num_of_vars, const tMatrixXd &A,
                              const tVectorXd &b, tVectorXd &x)
{
    x = tVectorXd::Ones(num_of_vars);
    double fast_left_min, fast_perp_vio;
    {
        if (num_of_vars == mOldX.size())
            x = mOldX;
        mSolver->lcp_fast(A, b, &x, 1e-13);
    }
    x = x.segment(0, num_of_vars).eval();
    // std::cout << "sol size = " << x.size() << std::endl;
    // std::cout << "num of vars = " << num_of_vars << std::endl;
    // std::cout << "res = " << x.transpose() << std::endl;
    calc_min(fast_left_min, fast_perp_vio, A, b, x);
    bool fast_fail = judge_fail(fast_left_min, fast_perp_vio);
    if (fast_fail)
    {
        // printf("[warn] moby fast left min %.5f, perp vio %.5f\n", fast_left_min,
        //        fast_perp_vio);
        mSolver->lcp_lemke(A, b, &x, 1e-10);
        // std::cout << "[warn] lcp fast failed, degenerate to lemke solver\n";
        // std::ofstream fout("output.txt");
        // fout << "A = \n" << A << std::endl;
        // fout << "b = " << b.transpose() << std::endl;
        // std::cout << "print A and b to output.txt";
        // exit(0);
    }
    mOldX = x;
    return 0;
}

void GetLCP(const int size, tMatrixXd &M, tVectorXd &q)
{
    M = tMatrixXd::Zero(size, size);
    M.setRandom();
    M = M.transpose() * M;
    q = tVectorXd::Random(size);

    // offical test case
    // M.fill(1);
    // q.fill(-1);
}

void calc_min(double &left_min, double &perp_vio, const tMatrixXd &A,
              const tVectorXd &b, const tVectorXd &sol)
{
    // printf("A size = %d %d\n", A.rows(), A.cols());
    // printf("b size = %d \n", b.size());
    // printf("sol size = %d \n", sol.size());
    left_min = std::min(sol.minCoeff(), (A * sol + b).minCoeff());
    perp_vio = std::fabs(sol.dot(A * sol + b));
}
bool judge_fail(double left_min, double perp_vio)
{
    return (left_min < -1e-6 || std::fabs(perp_vio) > 1e-6);
}