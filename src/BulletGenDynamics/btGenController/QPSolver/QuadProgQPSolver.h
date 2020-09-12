#pragma once
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
class QuadProgQPSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    QuadProgQPSolver();
    virtual bool Solve(int num_of_vars, tMatrixXd &H, tVectorXd &f,
                       tMatrixXd &Aeq, tVectorXd &beq, tMatrixXd &Aineq,
                       tVectorXd &bineq, int iters, tVectorXd &result);
};