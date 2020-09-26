#include "BulletGenDynamics/btGenUtil/MathUtil.h"
// #include <tuple>

class btGenFrameByFrameEnergyTerm
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenFrameByFrameEnergyTerm(int size_of_solution_vector);
    virtual int GetNumOfEnergy() const;
    virtual void AddEnergy(const tMatrixXd &Jacobian, const tVectorXd &residual,
                           double coeff, int st_pos, const std::string &name);
    virtual void GetEnergyTerm(tMatrixXd &A, tVectorXd &b);
    virtual void CheckEnergyValue(const tVectorXd &sol);

protected:
    void Validate(const tMatrixXd &jac, const tVectorXd &tVectorXd, int st);
    const int mSolutionSize;
    std::vector<std::string> mNameList;
    std::vector<double> mCoefList;
    tEigenArr<tMatrixXd> mAList;
    tEigenArr<tVectorXd> mbList;
    std::vector<int> mStartPosList;
    int mRows;
    // tMatrixXd mA;
    // tVectorXd mb;
};