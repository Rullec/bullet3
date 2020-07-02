#include "../../ExampleBrowser/ID_test/MathUtil.h"

// typedef Eigen::SparseMatrix<double> sparse_mat;

class cMatlabQPSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cMatlabQPSolver();
    void Solve(int num_of_vars, tMatrixXd&H, tVectorXd&f, tMatrixXd&Aeq, tVectorXd&beq, tMatrixXd&Aineq,
               tVectorXd&bineq, int iters, tVectorXd&result);
    // void SparseSolve(int num_of_vars, sparse_tMatrixXd& H, tVectorXd& f, tMatrixXd& Aeq, vec
    // & beq, tMatrixXd& Aineq, tVectorXd& bineq, int iters);
};