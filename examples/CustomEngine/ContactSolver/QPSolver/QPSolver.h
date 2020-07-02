#pragma once
#include "../../ExampleBrowser/ID_test/MathUtil.h"

class cQPSolver
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	cQPSolver();
	~cQPSolver();
	int Solve(int n, const tMatrixXd& H_, const tVectorXd f, const tMatrixXd& Aeq, const tVectorXd beq, const tMatrixXd& Aineq, const tVectorXd bineq, tVectorXd& x);

protected:
	tMatrixXd mH_buf;
	tVectorXd mf_buf;
};
