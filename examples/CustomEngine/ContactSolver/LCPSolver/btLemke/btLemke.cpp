#include "btLemke.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "btBulletDynamicsCommon.h"
#include <iostream>

btMatrixXu ConverttMatrixXdTobtMatrixX(const tMatrixXd& mat);

cBulletLemkeLCPSolver::cBulletLemkeLCPSolver() : cLCPSolverBase(eLCPSolverType::BulletLemke)
{
	solver = new btLemkeSolver();
	// solver = new btDantzigSolver();
}

/**
 * \brief			Solve 0 <= x \perp M * x + n >=0
*/
int cBulletLemkeLCPSolver::Solve(int num_of_vars, const tMatrixXd& M, const tVectorXd& n, tVectorXd& x)
{
	// Test();
	// btMatrixXu a = ConverttMatrixXdTobtMatrixX(M);
	// btVectorXu vec;
	//test
	// {
	// 	int num_of_vars = 13;
	// 	tMatrixXd M = tMatrixXd::Zero(num_of_vars, num_of_vars);
	// 	tVectorXd n = tVectorXd::Zero(num_of_vars);
	// 	tVectorXd x = tVectorXd::Zero(num_of_vars);
	// 	M.setRandom();
	// 	M = M.cwiseAbs2();
	// 	n.setRandom();
	// 	n *= -1;

	// 	M_buf = cBulletUtil::tMatrixXdTobtMatrixX(M);
	// 	n_buf = cBulletUtil::tVectorXdTobtVectorXd(-1 * n);
	// 	x_buf.resize(x.size());

	// 	lo_buf.resize(x.size());
	// 	hi_buf.resize(x.size());

	// 	for (int i = 0; i < x.size(); i++)
	// 	{
	// 		lo_buf[i] = -10;
	// 		hi_buf[i] = 10;
	// 	}
	// 	btAlignedObjectArray<int> limitDependency;
	// 	limitDependency.clear();
	// 	int numIterations = 1e4;
	// 	bool useSparsity = false;
	// 	solver->m_useLoHighBounds = false;
	// 	bool succ = solver->solveMLCP(M_buf, n_buf, x_buf, lo_buf, hi_buf, limitDependency, numIterations, useSparsity);

	// 	tVectorXd lo = cBulletUtil::btVectorXdTotVectorXd(lo_buf), hi = cBulletUtil::btVectorXdTotVectorXd(hi_buf);
	// 	x = cBulletUtil::btVectorXdTotVectorXd(x_buf);
	// 	std::cout << "M = \n"
	// 			  << M << std::endl;
	// 	std::cout << "n = \n"
	// 			  << n.transpose() << std::endl;
	// 	std::cout << "lo = " << lo.transpose() << std::endl;
	// 	std::cout << "hi = " << hi.transpose() << std::endl;
	// 	std::cout << "x = " << x.transpose() << std::endl;
	// 	std::cout << "M*x+n = " << (M * x + n).transpose() << std::endl;
	// 	std::cout << "solved status " << succ << std::endl;
	// 	exit(1);
	// }

	// we need to test succ mannually here....It's hard to understand
	M_buf = cBulletUtil::tMatrixXdTobtMatrixX(M);
	n_buf = cBulletUtil::tVectorXdTobtVectorXd(n);
	// n_buf = cBulletUtil::tVectorXdTobtVectorXd(n);
	x_buf.resize(x.size());

	lo_buf.resize(x.size());
	hi_buf.resize(x.size());

	for (int i = 0; i < x.size(); i++)
	{
		lo_buf[i] = 0;
		hi_buf[i] = 1e4;
	}
	btAlignedObjectArray<int> limitDependency;
	limitDependency.clear();
	int numIterations = solver->m_maxLoops;
	bool useSparsity = false;
	solver->m_useLoHighBounds = true;
	bool succ = solver->solveMLCP(M_buf, n_buf, x_buf, lo_buf, hi_buf, limitDependency, numIterations, useSparsity);

	tVectorXd lo = cBulletUtil::btVectorXdTotVectorXd(lo_buf), hi = cBulletUtil::btVectorXdTotVectorXd(hi_buf);
	x = cBulletUtil::btVectorXdTotVectorXd(x_buf);
	// std::cout << "M = \n"
	// 		  << M << std::endl;
	// std::cout << "n = \n"
	// 		  << n.transpose() << std::endl;
	// std::cout << "lo = " << lo.transpose() << std::endl;
	// std::cout << "hi = " << hi.transpose() << std::endl;
	// std::cout << "x = " << x.transpose() << std::endl;
	// std::cout << "M*x-n = " << (M * x - n).transpose() << std::endl;
	std::cout << "solved status " << succ << std::endl;
	// exit(1);

	return 0;
}

// tMatrixXd ConvertbtMatrixXd

void cBulletLemkeLCPSolver::Test()
{
	// 	int num_of_vars = 10;
	// 	tMatrixXd M = tMatrixXd::Zero(num_of_vars, num_of_vars);
	// 	tVectorXd n = tVectorXd::Zero(num_of_vars);
	// 	tVectorXd x = tVectorXd::Zero(num_of_vars);
	// 	M.setRandom();
	// 	n.setRandom();

	// 	// std::cout << "residual = " << (M - ConvertbtMatrixXTotMatrixXd(ConverttMatrixXdTobtMatrixX(M))) << std::endl;
	// 	// std::cout << "residual = " << (n - ConverbtVectorXdTotVectorXd(ConvertVectorXdTobtVectorXd(n))) << std::endl;

	// 	exit(1);
	// 	// Solve(num_of_vars, M, n, x);
}