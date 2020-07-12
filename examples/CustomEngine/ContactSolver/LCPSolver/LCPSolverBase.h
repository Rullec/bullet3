#pragma once
#include "../../../ExampleBrowser/ID_test/MathUtil.h"

enum eLCPSolverType
{
	NativeLemke,
	BulletLemke,
	BulletDantzig,
	NUM_LCP_SOLVER
};

class cLCPSolverBase
{
public:
	cLCPSolverBase(eLCPSolverType type);
    
	eLCPSolverType GetType();
	virtual int Solve(int num_of_vars, const tMatrixXd& A, const tVectorXd& b, tVectorXd& x) = 0;

protected:
	eLCPSolverType mType;
};