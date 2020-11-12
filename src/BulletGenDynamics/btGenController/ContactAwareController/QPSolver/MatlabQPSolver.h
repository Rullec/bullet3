#pragma once
#include "QuadProgQPSolver.h"

class MatlabQPSolver : public QuadProgQPSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MatlabQPSolver();
    virtual bool Solve(int num_of_vars, tMatrixXd &H, tVectorXd &f,
                       tMatrixXd &Aeq, tVectorXd &beq, tMatrixXd &Aineq,
                       tVectorXd &bineq, int iters,
                       tVectorXd &result) override final;
    // void SparseSolve(int num_of_vars, sparse_mat & H, tVectorXd & f,
    // tMatrixXd & Aeq, tVectorXd & beq, tMatrixXd & Aineq, tVectorXd & bineq,
    // int iters);
};