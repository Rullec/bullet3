#include "../LCPSolverBase.h"
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include "btBulletDynamicsCommon.h"

// class btDantzigSolver;
class cBulletPathLCPSolver : public cLCPSolverBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cBulletPathLCPSolver();
    virtual int Solve(int num_of_vars, const tMatrixXd &M, const tVectorXd &n,
                      tVectorXd &x) override final;

protected:
    // int SolveBybtLemke(int num_of_vars, const tMatrixXd& M, const tVectorXd&
    // n, tVectorXd& x);
    void Test();

    // btLemkeSolver * solver;
    // // btDantzigSolver * solver;
    // btMatrixXu M_buf;
    // btVectorXu n_buf, x_buf, lo_buf, hi_buf;
};
