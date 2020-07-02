#include "QPSolver.h"
#include "EigenQP.h"
#include "uQuadProg++.hh"
#include <Eigen/Eigen>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/progress.hpp>
#include <iostream>
#include <sstream>
#include <string>

using boost::timer;

using namespace Eigen;
namespace ublas = boost::numeric::ublas;
using std::cout;
using std::endl;

cQPSolver::cQPSolver()
{
}

cQPSolver::~cQPSolver()
{
}

/*	
	solve: 
	func: 0.5 * x.T * H * x + f.T * x
	s.t. Aeq.T * x + beq = 0
	s.t. Aineq.T * x + bineq >=0
*/
int cQPSolver::Solve(int n, const tMatrixXd& H, const tVectorXd f, const tMatrixXd& Aeq, const tVectorXd beq, const tMatrixXd& Aineq, const tVectorXd bineq, tVectorXd& x)
// int cQPSolver::Solve()
{
	// check input
	if (x.size() != n)
	{
		std::cout << "x size = " << x.size() << "!=" << n << ", exit\n";
		exit(1);
	}

	if (H.rows() != n || H.cols() != n)
	{
		std::cout << "H.shape = " << H.rows() << " " << H.cols() << " doesn't match n = " << n << std::endl;
		exit(1);
	}

	int num_eqs = Aeq.cols(),
		num_ineqs = Aineq.cols();
	if (Aeq.rows() != n)
	{
		std::cout << "Aeq rows " << Aeq.rows() << " != " << n << std::endl;
		exit(1);
	}
	if (num_eqs != beq.size())
	{
		std::cout << "Aeq size " << num_eqs << " doesn't match beq = " << beq.size() << std::endl;
		exit(1);
	}
	if (Aineq.rows() != n)
	{
		std::cout << "Aineq rows " << Aineq.rows() << " != " << n << std::endl;
		exit(1);
	}

	if (num_ineqs != bineq.size())
	{
		std::cout << "Aineq size " << num_ineqs << " doesn't match bineq = " << bineq.size() << std::endl;
		exit(1);
	}

	mH_buf = H;
	mf_buf = f;
	QP::solve_quadprog(mH_buf, mf_buf, Aeq, beq, Aineq, bineq, x);
	// {
	// 	boost::timer clock;
	// 	int n = 60;
	// 	int n_eq = 0, n_ineq = 120;
	// 	tMatrixXd A, Aeq, Aineq;
	// 	tVectorXd b, beq, bineq, x;
	// 	A.resize(n, n);
	// 	A.setRandom();
	// 	// A.setIdentity();
	// 	b.resize(n);
	// 	b.setRandom();

	// 	tMatrixXd H;
	// 	tVectorXd f;
	// 	// H = 2 * A.transpose() * A;
	// 	H = 2 * A.transpose() * A;
	// 	// std::cout << H.rows() << H.cols() << std::endl;
	// 	f = 2 * A.transpose() * b;

	// 	Aeq.resize(n, n_eq);
	// 	Aeq.setIdentity();
	// 	Aineq.resize(n, n_ineq);
	// 	Aineq.setRandom();
	// 	beq = tVectorXd::Random(n_eq);
	// 	bineq = tVectorXd::Random(n_ineq);
	// 	tMatrixXd old_H = H;
	// 	tVectorXd old_f = f;
	// 	clock.restart();
	// 	QP::solve_quadprog(old_H, old_f, Aeq, beq, Aineq, bineq, x);

	// 	std::cout << "Aineq = \n"
	// 			  << Aineq << std::endl;
	// 	std::cout << "bineq = \n"
	// 			  << bineq.transpose() << std::endl;
	// 	std::cout << "x = " << x.transpose() << std::endl;
	// 	std::cout << "Aineq * x + bineq = " << (Aineq.transpose() * x + bineq).transpose() << std::endl;
	// 	std::cout << "time cost " << clock.elapsed() << std::endl;
	// }
	return 0;
}