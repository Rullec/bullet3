#include "../../ExampleBrowser/ID_test/BulletUtil.h"
#include "../LCPSolverBase.h"
#include "btBulletDynamicsCommon.h"

class btLemkeSolver;
class cBulletLemkeLCPSolver : public cLCPSolverBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	cBulletLemkeLCPSolver();
	virtual int Solve(int num_of_vars, const tMatrixXd& M, const tVectorXd& n, tVectorXd& x) override final;

protected:
	// int SolveBybtLemke(int num_of_vars, const tMatrixXd& M, const tVectorXd& n, tVectorXd& x);
	void Test();

	btLemkeSolver * solver;
	// btDantzigSolver * solver;
	btMatrixXu M_buf;
	btVectorXu n_buf, x_buf, lo_buf, hi_buf;

};
