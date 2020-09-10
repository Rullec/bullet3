#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"

class btTraj;
class btGeneralizeWorld;
class cRobotModelDynamics;
class QuadProgQPSolver;
class btCharContactPt;
class btGenFrameByFrameConstraint;
class btGenFrameByFrameEnergyTerm;
enum eContactStatus
{
	INVALID_CONTACT_STATUS,
	SLIDING,
	STATIC,
	BREAKAGE
};
class btGenFrameByFrameOptimizer
{
public:
	btGenFrameByFrameOptimizer();
	~btGenFrameByFrameOptimizer();
	void Init(const Json::Value& conf, btTraj* ref_traj, btGeneralizeWorld* world);
	void CalcTarget(double dt, int target_frame_id, tVectorXd& tilde_qddot, tVectorXd& tilde_tau);

protected:
	// ---------methods
	void ParseConfig(const Json::Value& conf);
	void InitModelInfo();
	void InitQPSolver();
	void CalcContactStatus();
	void CalcSolutionVector();
	void CalcEnergyTerms();
	void CalcConstraints();
	void Solve(tVectorXd& tilde_qddot, tVectorXd& tilde_tau);
	void CalcTargetInternal(const tVectorXd& solution, tVectorXd& qddot, tVectorXd& tau);
	void ClearContactPoints();

	void AddSlidingConstraint(btCharContactPt* pt);
	void AddStaticConstraint(btCharContactPt* pt);
	void AddBreakageConstraint(btCharContactPt* pt);
	void AddDynamicEnergyTerm();
	void AddDynamicEnergyTermPos();
	void AddDynamicEnergyTermVel();
	void AddDynamicEnergyTermAccel();
	void AddDynamicConstraint();
	void AddMinTauEnergyTerm();
	void AddMinContactForceEnergyTerm();
	void CalcContactConvertMat(btCharContactPt* contact, tMatrixXd& convert_mat);
	int GetSolutionSizeByContactStatus(eContactStatus status);

	// ---------vars
	int mCurFrameId;
	double mdt;
	QuadProgQPSolver* mQPSolver;
	cRobotModelDynamics* mModel;
	btTraj* mTraj;
	btGeneralizeWorld* mWorld;
	std::vector<btCharContactPt*> mContactPoints;
	std::vector<int> mContactSolOffset;  // record the offset of each contact point in solution vector
	std::vector<int> mContactSolSize;    // record the occupied "size" w.r.t each contact pt in solution vector

	// optimization vars
	bool mEnableHardConstraintForDynamics;
	bool mUseNativeRefTarget;
	double mDynamicPosEnergyCoeff;
	double mDynamicVelEnergyCoeff;
	double mDynamicAccelEnergyCoeff;
	const double mu = 0.8;
	int mContactSolutionSize;  // the size of contact force in result vector
	int mCtrlSolutionSize;     // the size of active ctrl froce in result vector
	int mTotalSolutionSize;    // total solution size

	tMatrixXd A,  // E = x^T A x + x^T b
		Aeq,
		Aineq;
	tVectorXd b, beq, bineq;
	btGenFrameByFrameConstraint* mConstraint;
	btGenFrameByFrameEnergyTerm* mEnergyTerm;

	// model buffer vars
	int num_of_freedom;
	int num_of_underactuated_freedom;
};