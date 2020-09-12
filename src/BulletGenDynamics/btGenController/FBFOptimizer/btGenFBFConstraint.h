#include "BulletGenDynamics/btGenUtil/MathUtil.h"
// #include <tuple>

class btGenFrameByFrameConstraint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenFrameByFrameConstraint(int size_of_solution_vector);
    virtual int GetNumOfEqCon() const;
    virtual int GetNumOfIneqCon() const;
    virtual void AddEqCon(const tMatrixXd &Jacobian, const tVectorXd &residual,
                          int st_pos);
    virtual void AddEquivalentEqCon(const tMatrixXd &Jacobian,
                                    const tVectorXd &residual, int st_pos,
                                    const double tolerance);

    // Jacobian * x + residual >= 0
    virtual void AddIneqCon(const tMatrixXd &Jacobian,
                            const tVectorXd &residual, int st_pos);
    virtual void GetEqJacobianAndResidual(tMatrixXd &Jac, tVectorXd &res);
    virtual void GetIneqJacobianAndResidual(tMatrixXd &Jac, tVectorXd &res);

protected:
    void Validate(const tMatrixXd &jac, const tVectorXd &tVectorXd, int st);
    const int mSolutionSize;
    int mNumOfEqConstraint;
    std::vector<int> mEqConstraintStPos; // where to put the constraint jacobian
    tEigenArr<tVectorXd> mEqConstraintResidual; // Jacobian * x - residual = 0
    tEigenArr<tMatrixXd> mEqConstraintJacobian;

    int mNumOfIneqConstraint;
    std::vector<int>
        mIneqConstraintStPos; // where to put the constraint jacobian
    tEigenArr<tVectorXd> mIneqConstraintResidual; // Jacobian * x - residual < 0
    tEigenArr<tMatrixXd> mIneqConstraintJacobian;
};