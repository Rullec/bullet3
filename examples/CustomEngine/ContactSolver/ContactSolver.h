#include "../../ExampleBrowser/ID_test/BulletUtil.h"
#include "../../ExampleBrowser/ID_test/MathUtil.h"
#include "ConstraintData.h"
#include <memory>

class btDiscreteDynamicsWorld;
class btPersistentManifold;
class cSimRigidBody;
class cLCPSolver;
// class cQPSolver;
class cMatlabQPSolver;
struct cRobotCollider;
struct tCollisionObjData;
class cContactSolver : public std::enable_shared_from_this<cContactSolver>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	struct tParams
	{
		int mNumFrictionConeDirs;
		double mMu;
		btDiscreteDynamicsWorld* mWorld;
		tParams();
	};

	cContactSolver(const tParams& params);
	virtual ~cContactSolver();

	void ConstraintProcess(float dt);

protected:
	int mNumFrictionDirs;
	double mMu;
	float dt;
	bool mEnableMultibodySelfCol;	// enable the self-collision of multibody 
	bool mEnableTest;
	btDiscreteDynamicsWorld* mWorld;
	cLCPSolver* mLCPSolver;
	// cQPSolver * mQPSolver;
	cMatlabQPSolver * mMatlabQPSolver;
	std::vector<int> mMap_collisionid_to_groupid;
	std::vector<tCollisionObjData*> mColGroupData;
	std::vector<tContactPointData*> mContactConstraintData;

	// 1. LCP buffer vars, rel vel buffer: cartesian force to cartesian rel velocity
	tMatrixXd rel_vel_convert_mat;
	tVectorXd rel_vel_convert_vec;

	// 2. LCP buffer vars, normal and tangent convert mat buffer, cartesian force to normal and tangent rel vel
	tMatrixXd normal_vel_convert_mat;	// one contact point one line
	tVectorXd normal_vel_convert_vec;	// one contact point one real number
	tMatrixXd tangent_vel_covert_mat;	// one contact point N lines, N = mNumFrictionDir
	tVectorXd tangent_vel_convert_vec;	// one contact point one real number
	
	// 3. LCP buffer vars, from result vector to normal/tangent velcoity (ultimate convert mat)
	tMatrixXd normal_vel_convert_result_based_mat;
	tVectorXd normal_vel_convert_result_based_vec;
	tMatrixXd tangent_vel_covert_result_based_mat;
	tVectorXd tangent_vel_convert_result_based_vec;
	
	// LCP condition: x \perp (M * x + n)
	tMatrixXd M;
	tVectorXd x, n, x_qp;

	void ConstraintSetup();
	void ConstraintSolve();
	void ConstraintSolveByQP();
	void ConstraintSolveByMatlabQP();
	void UpdateVelocity(float dt);
	void ConstraintFinished();
	void CalcCMats(tMatrixXd& C_lambda, tMatrixXd& C_mufn, tMatrixXd& C_c);
	void RebuildColObjData();
	void DeleteColObjData();
	void DeleteContactData();
	void AddManifold(btPersistentManifold* mani);

	void CalcAbsvelConvertMat();			// cartesian force -> contact point abs vel
	void CalcRelvelConvertMat();			// cartesian force -> contact point rel vel
	void CalcDecomposedConvertMat();		// cartesian force -> contact point normal/tangent rel vel
	void CalcResultVectorBasedConvertMat();	// result vector -> contact point normal/tangent rel vel

	// Test cartesian force to cartesian velcoity convert mat
	void PushState();
	void PopState();
	void TestCartesianForceToCartesianVel();
	void TestCartesianForceToCartesianRelVel(const tMatrixXd& convert_mat, const tVectorXd& convert_vec);
	void TestCartesianForceToNormalAndTangetRelVel(const tMatrixXd& normal_mat, const tVectorXd& normal_vec, const tMatrixXd& tan_mat, const tVectorXd& tan_vec);
	void TestCartesianForceToNormalAndTangetResultBasedRelVel(const tMatrixXd& normal_mat, const tVectorXd& normal_vec, const tMatrixXd& tan_mat, const tVectorXd& tan_vec);
};