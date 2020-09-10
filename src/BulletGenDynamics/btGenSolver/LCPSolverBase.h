#pragma once
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

enum eLCPSolverType
{
	NativeLemke,
	BulletLemke,
	BulletDantzig,
	ODEDantzig,
	NUM_LCP_SOLVER
};

class cLCPSolverBase
{
public:
	cLCPSolverBase(eLCPSolverType type);
	virtual ~cLCPSolverBase() = default;
	eLCPSolverType GetType();
	virtual int Solve(int num_of_vars, const tMatrixXd& A, const tVectorXd& b, tVectorXd& x) = 0;

protected:
	eLCPSolverType mType;
};