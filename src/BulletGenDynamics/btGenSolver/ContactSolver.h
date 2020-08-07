#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "ConstraintData.h"

class btDiscreteDynamicsWorld;
class btPersistentManifold;
class cRigidBody;
// class cNativeLemkeLCPSolver;
// class cQPSolver;
// class cMatlabQPSolver;
class cLCPSolverBase;
struct cRobotCollider;
struct tCollisionObjData;
class cRobotModelDynamics;
struct tContactForce
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	tContactForce(cCollisionObject* mObj,
				  const tVector& mForce, const tVector& mWorldPos);
	cCollisionObject* mObj;
	tVector mForce, mWorldPos;  // position in world and force in world frame
};
struct tConstraintGeneralizedForce
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	tConstraintGeneralizedForce(cRobotModelDynamics* _model, int _dof_id, double _val) : model(_model), dof_id(_dof_id), value(_val)
	{
	}

	cRobotModelDynamics* model;
	double dof_id;
	double value;
};

class cContactSolver
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cContactSolver(const std::string& config_path, btDiscreteDynamicsWorld* world);
	virtual ~cContactSolver();

	void ConstraintProcess(float dt);
	bool GetEnableConvertMatTest() { return mEnableConvertMatTest; }
	std::vector<tContactForce*> GetContactForces();
	std::vector<tConstraintGeneralizedForce*> GetConstraintGeneralizedForces();

protected:
	int mNumFrictionDirs;
	double mMu;
	float cur_dt;
	bool mEnableMultibodySelfCol;  // enable the self-collision of multibody
	bool mEnableConvertMatTest;
	bool mEnableDebugOutput;

	// lcp config
	bool mEnableLCPCalc;
	bool mEnableContactLCP;
	bool mEnableFrictionalLCP;
	bool mEnableJointLimitLCP;
	bool mUseLCPResult;

	// SI config
	bool mEnableSICalc;
	bool mEnableFrictionalSI;
	bool mUseSIResult;
	int mMaxItersSI;
	double mConvergeThresholdSI;

	bool mEnableSILCPComparision;  // compare the result of SI and LCP
	double mDiagonalEps;

	btDiscreteDynamicsWorld* mWorld;
	cLCPSolverBase* mLCPSolver;

	std::vector<int> map_colobjid_to_groupid;
	std::vector<tCollisionObjData*> mColGroupData;
	std::vector<tContactPointData*> mContactConstraintData;
	std::vector<tJointLimitData*> mJointLimitConstraintData;
	std::vector<cRobotModelDynamics*> mMultibodyArray;
	int mNumContactPoints;
	int mNumConstraints;
	int mNumJointLimitConstraints;

	// 1. LCP buffer vars, rel vel buffer: cartesian force to cartesian rel velocity
	tMatrixXd rel_vel_convert_mat;
	tVectorXd rel_vel_convert_vec;

	// 2. LCP buffer vars, normal and tangent convert mat buffer, cartesian force to normal and tangent rel vel
	tMatrixXd normal_vel_convert_mat;   // one contact point one line
	tVectorXd normal_vel_convert_vec;   // one contact point one real number
	tMatrixXd tangent_vel_convert_mat;  // one contact point N lines, N = mNumFrictionDir
	tVectorXd tangent_vel_convert_vec;  // one contact point one real number

	// 3. LCP buffer vars, from result vector to normal/tangent velcoity (ultimate convert mat)
	tMatrixXd normal_vel_convert_result_based_mat;
	tVectorXd normal_vel_convert_result_based_vec;
	tMatrixXd tangent_vel_convert_result_based_mat;
	tVectorXd tangent_vel_convert_result_based_vec;

	// LCP condition: x \perp (M * x + n)
	tMatrixXd M;
	tVectorXd x_lcp, n;
	tVectorXd f_lcp;  // cartesian contact forces array for LCP problem

	// Sequential impulse solve result x_si
	tVectorXd x_si;

	// contact forces buffer
	std::vector<tContactForce*> contact_force_array;
	std::vector<tConstraintGeneralizedForce*> contact_torque_array;

	void ConstraintSetup();
	void ConstraintSolve();
	void SolveByLCP();
	void SolveBySI();
	void ConvertLCPResult();
	double InternalIterationSI();
	void SetupDataForSI();
	void UpdateDataForSI();
	void CompareSILCPResult();

	void UpdateVelocity(float dt);
	void ClearAllConstraintForce();
	void ConstraintFinished();
	void CalcCMats(tMatrixXd& C_lambda, tMatrixXd& C_mufn, tMatrixXd& C_c);
	void CollectMultibody();
	void RebuildColObjData();
	void DeleteColObjData();
	void DeleteConstraintData();
	void AddManifold(btPersistentManifold* mani);
	void AddJointLimit();

	void CalcAbsvelConvertMat();             // cartesian force -> contact point abs vel
	void CalcRelvelConvertMat();             // cartesian force -> contact point rel vel
	void CalcDecomposedConvertMat();         // cartesian force -> contact point normal/tangent rel vel
	void CalcResultVectorBasedConvertMat();  // result vector -> contact point normal/tangent rel vel

	// Test cartesian force to cartesian velcoity convert mat
	void PushState(const std::string& name);
	void PopState(const std::string& name);
	void TestCartesianForceToCartesianVel();
	void TestCartesianForceToCartesianRelVel(const tMatrixXd& convert_mat, const tVectorXd& convert_vec);
	void TestCartesianForceToNormalAndTangetRelVel(const tMatrixXd& normal_mat, const tVectorXd& normal_vec, const tMatrixXd& tan_mat, const tVectorXd& tan_vec);
	void TestCartesianForceToNormalAndTangetResultBasedRelVel(const tMatrixXd& normal_mat, const tVectorXd& normal_vec, const tMatrixXd& tan_mat, const tVectorXd& tan_vec);
	void TestSICartesianConvertMatAndVec();
	bool IsMultibodyAndVelMax(cCollisionObject* body);
};