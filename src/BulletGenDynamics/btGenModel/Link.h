#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H

#include "BaseObject.h"

class Link : public BaseObject
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	explicit Link(BaseObjectParams& param);
	explicit Link(BaseObjectJsonParam& param);

	void Tell() override;

	void UpdateState(bool compute_gradient) override;
	void UpdateMeshMatrix() override;
	void InitTerms() override;
	bool IsJoint() const override { return false; }

	void UpdateMWQ();
	tMatrix& GetMWQQ(int i, int j) override;

	//===============For RigidBody Dynamics===============
	void ComputeMassMatrix() override;
	void ComputeJKv_dot(tVectorXd& q_dot, tVector3d& p) override;
	void ComputeJKw_dot(tVectorXd& q_dot) override;
	void ComputeDJkvdq(tVector3d& p) override;
	void ComputeDJkwdq() override;
	//====================================================

	//===================For Simulation===================
	void SetLinkVel(const tVector3d& link_vel);
	void SetLinkOmega(const tVector3d& link_omega);
	tVector3d GetLinkVel() const;
	tVector3d GetLinkOmega() const;
	double GetLinkMaxLength() const;

protected:
	tVector3d link_vel, link_omega;  // the lin vel and ang vel w.r.t the COM of this link in world frame
};

#endif  //ROBOT_LINK_H
