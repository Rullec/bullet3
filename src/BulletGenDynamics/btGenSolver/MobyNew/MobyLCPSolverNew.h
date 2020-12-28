#pragma once
#include "../LCPSolverBase.h"

// class moby_lcp_solver;
class UnrevisedLemkeSolver;
class btGenMobyLCPSovlerNew : public cLCPSolverBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    btGenMobyLCPSovlerNew();
    ~btGenMobyLCPSovlerNew();

    virtual int Solve(int num_of_vars, const tMatrixXd &A, const tVectorXd &b,
                      tVectorXd &x);

private:
    UnrevisedLemkeSolver *mSolver;
};
