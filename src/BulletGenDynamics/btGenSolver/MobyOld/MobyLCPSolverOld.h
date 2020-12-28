#pragma once
#include "../LCPSolverBase.h"

class moby_lcp_solver;
class btGenMobyLCPSovler : public cLCPSolverBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    btGenMobyLCPSovler();
    ~btGenMobyLCPSovler();

    virtual int Solve(int num_of_vars, const tMatrixXd &A, const tVectorXd &b,
                      tVectorXd &x);

private:
    moby_lcp_solver *mSolver;
    tVectorXd mOldX;
};
