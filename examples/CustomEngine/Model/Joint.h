#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H
#include <vector>
#include "ModelEigenUtils.h"
#include "BaseObject.h"

class Joint : public BaseObject
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	explicit Joint(BaseObjectParams& param);
	explicit Joint(BaseObjectJsonParam& param);

	void Tell() override;
	Freedom* AddFreedom(Freedom& f) override;
	int GetNumOfFreedom() override;
	void SetFreedomValue(int id, double v) override;
	void GetFreedomValue(int id, double& v) override;

	void SetFreedomValue(std::vector<double>& v) override;
	void GetFreedomValue(std::vector<double>& v) override;

	void CleanGradient() override;

	Freedom* GetFreedoms(int order) override;
	Freedom* GetFreedomByAxis(tVector3d axis, int type = REVOLUTE) override;

	void InitTerms() override;
	void InitMatrix();

	bool IsJoint() const override { return true; }
	void UpdateState(bool compute_gradient) override;
	void UpdateMatrix();
	void ComputeLocalFirstDeriveMatrix();
	void ComputeLocalSecondDeriveMatrix();
	virtual void GetRotations(tMatrix3d& m);
	virtual void ComputeTransformFirstDerive();
	void ComputeGlobalTransformFirstDerive();

	virtual void ComputeLocalTransformSecondDerive();
	void ComputeGlobalTransformSecondDerive();

	void ComputeJacobiByGivenPointTotalDOF(tMatrixXd& j, tVector p) override;
	void ComputeJacobiByGivenPoint(tMatrixXd& j, tVector& p) override;
	void ComputeHessianByGivenPoint(EIGEN_V_MATXD& ms, tVector& p) override;
	void ComputeJKv_dot(tVectorXd& q_dot, tVector3d& p) override;
	void ComputeJKw_dot(tVectorXd& q_dot) override;

	void ComputeLocalTransform() override;
	void ComputeGlobalTransform() override;

	tMatrix& GetMWQQ(int i, int j) override;

	const tMatrixXd& GetJKDot() const override;

protected:
	std::vector<Freedom> freedoms;  // self freedom. For revolute it is 1, for spherical it is 3.

	EIGEN_V_tMatrixD r_m;
	EIGEN_V_tMatrixD r_m_first_deriv;
	EIGEN_V_tMatrixD r_m_second_deriv;

	// ===========second derive==============
	EIGEN_VV_tMatrixD mTqq;
	EIGEN_VV_tMatrixD mWqq;
	// ======================================


	// buffer used in Jacobian computation
	tMatrixXd local_jac_buf;
	// tMatrixXd global_jac_buf;
};

#endif  //ROBOT_JOINT_H
