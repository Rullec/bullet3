#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class cRobotModelDynamics;
class btGenPDController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	btGenPDController(const std::string & config);
	virtual ~btGenPDController();
	bool Init(cRobotModelDynamics* model);
	void SetPDTargetq(const tVectorXd& q);
	void SetPDTargetqdot(const tVectorXd& qdot);
	void ApplyGeneralizedTau(double timestep);

protected:
	cRobotModelDynamics * mModel;
	double mTorqueLim;		// max torque limit for stability
	double mKpConstant, mKdConstant;
	tVectorXd mKp, mKd;		// PD coefs
	tVectorXd mTargetq, mTargetqdot;
	bool mEnableSPD;  // enable stable pd control
};