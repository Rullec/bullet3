#include "BulletGenDynamics/btGenUtil/MathUtil.h"
// #include <tuple>

class btGenFrameByFrameEnergyTerm
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	btGenFrameByFrameEnergyTerm(int size_of_solution_vector);
	virtual int GetNumOfEnergy() const;
	virtual void AddEnergy(const tMatrixXd &Jacobian, const tVectorXd &residual, int st_pos);
	virtual void GetEnergyTerm(tMatrixXd &A, tVectorXd &b);

protected:
	void Validate(const tMatrixXd &jac, const tVectorXd &tVectorXd, int st);
	const int mSolutionSize;
	tEigenArr<tMatrixXd> mAlist;
	tEigenArr<tVectorXd> mblist;
    std::vector<int> mStartPosList;
    int mRows;
	// tMatrixXd mA;
	// tVectorXd mb;
};