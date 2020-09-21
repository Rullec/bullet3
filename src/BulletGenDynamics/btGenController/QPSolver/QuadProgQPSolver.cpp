#include "QuadProgQPSolver.h"
#include "eiquadprog.hpp"
#include <iostream>
using namespace Eigen;

QuadProgQPSolver::QuadProgQPSolver() {}

/**
 *
 *  min: 0.5 *x^T * H * x + f^T * x
 *  s.t. Aeq.T * x + beq = 0
 *       Aineq.T * x + bineq >=0
 */
bool QuadProgQPSolver::Solve(int num_of_vars, tMatrixXd &H, tVectorXd &f,
                             tMatrixXd &Aeq, tVectorXd &beq, tMatrixXd &Aineq,
                             tVectorXd &bineq, int iters, tVectorXd &result)
{
    result = tVectorXd::Zero(num_of_vars);
    // std::cout << "H size = " << H.rows() << " " << H.cols() << std::endl;
    // std::cout << "f size = " << f.rows() << " " << f.cols() << std::endl;
    // std::cout << "Aeq size = " << Aeq.rows() << " " << Aeq.cols() <<
    // std::endl; std::cout << "beq size = " << beq.rows() << " " << beq.cols()
    // << std::endl; std::cout << "Aineq size = " << Aineq.rows() << " " <<
    // Aineq.cols()
    //           << std::endl;
    // std::cout << "bineq size = " << bineq.rows() << " " << bineq.cols()
    //           << std::endl;
    // exit(0);
    // std::cout << "H = \n" << H << std::endl;
    // std::cout << "f = " << f.transpose() << std::endl;
    // tMatrixXd old_H = H;
    // tVectorXd old_f = f;
    double ret = solve_quadprog(H, f, Aeq, beq, Aineq, bineq, result);
    if (Aeq.rows() > 0)
    {
        tVectorXd eq_res = Aeq.transpose() * result + beq;
    }
    if (Aineq.rows() > 0)
    {
        tVectorXd ineq_res = Aineq.transpose() * result + bineq;
    }

    // std::cout << "eq res = " << eq_res.transpose() << std::endl;
    // std::cout << "ineq res = " << ineq_res.transpose() << std::endl;
    // std::cout << "result = " << result.transpose() << std::endl;
    // std::cout << "ret = " << ret << std::endl;
    // if (std::isinf(ret) == true)
    // {
    //     std::cout << "[error] solve QP failed\n";
    //     exit(0);
    // }
    // if (eq_res.norm() > 1e-5)
    // {
    //     std::cout << "[error] Aeq = \n" << Aeq.transpose() << std::endl;
    //     std::cout << "[error] beq = " << beq.transpose() << std::endl;
    //     exit(0);
    // }

    return true;
}