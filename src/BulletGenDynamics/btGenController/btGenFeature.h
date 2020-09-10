#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"

enum btGenFeatureType
{
	INVALID_FEATURE_TYPE,
	SphereJoint,
	RevoluteJoint,
	NoneJoint,
	FixedJoint,
	Height,
	Location,
	NUM_OF_GEN_FEATURE_TYPE
};

const std::string gGenFeatureTypeStr[] = {
	"INVALID_FEATURE_TYPE",
	"SphereJoint",
	"RevoluteJoint",
	"NoneJoint",
	"FixedJoint",
	"Height",    // Y axis
	"Location",  // X-Z axis
};

const int gGenFeatureSize[] = {-1, 3, 1, 6, 0, 1, 2};

/**
 * \brief			A btGenFeature is a consist element in the total feature vector.
 * 		
 * 		It can be:
 * 			1. joint angle
 * 			2. link location
 * 			3. link height
 * 		And we can also control the 
*/
struct btGenFeature
{
	int mLinkId;                    // the belonging link of this feature
	std::string mLinkName;          // link name is also dummy specified
	std::string mFeatureName;       // the name of this feature. format: linkname + suffix
	btGenFeatureType mFeatureType;  //
	int mFeatureOrder;              // order: 0-pos, 1-vel, 2-accel
	double mWeight;                 // feature weight
};

class cRobotModelDynamics;
class btTraj;

/**
 * \brief				This class construct the feature vector in contact-aware LCP controlk
*/
class btGenFeatureArray
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	// btGenFeatureArray(const Json::Value& conf);
	btGenFeatureArray();
	void Init(const std::string& conf, cRobotModelDynamics* model, btTraj* traj, const tVector& g);
	// void Eval(const int target_frame, tMatrixXd& H, tMatrixXd& E, tVectorXd& f);
	void Eval(const tVectorXd& qddot_target, const tVectorXd& tau_target, tMatrixXd& H, tMatrixXd& E, tVectorXd& f);
	~btGenFeatureArray();

protected:
	cRobotModelDynamics* mModel;
	// btTraj* mTraj;  // guided trajectory

	int mNumOfFeature;
	int mTotalFeatureSize;                      // the size of total controllable feature
	std::vector<btGenFeature*> mFeatureVector;  // feature vector definition
	std::vector<int> mFeatureOffset;
	tVectorXd mRefFeature;  // feature vector from the ref trajectory
	tVector mGravity;       // gravity accel
	tVectorXd mW, mWm;      // mW: feature vector weight, mWm: ref tau weight

	tMatrixXd mN;                               // convert the underacuated tau to full-DOF tau
	tMatrixXd mConvertPosToY, mConvertPosToXZ;  // two convert matrix whican can convert [x, y, z] to "Y" or "X,Z"
	tMatrixXd mC_alpha;                         // convert mat from qddot to feature vector
	tVectorXd md_alpha;                         // convert residual from qddot to feature vector
	tMatrixXd mD_tau, mM_Q;                     // convert mat from tau/Q_\chi to qddot, \chi is the contact force
	tVectorXd mn_tau;                           // convert residual from tau/Q_\chi to qddot, \chi is the contact force
	//--------------methods
	void InitWeight(const std::string& conf);
	void InitFeature(const std::string& conf);
	// void InitRefFeature();
	void EvalConvertMatAndResidual(tMatrixXd& C_alpha, tVectorXd& d_alpha) const;
	void EvalPrerequistite(tMatrixXd& D_tau, tMatrixXd& M_Q, tVectorXd& n_tau) const;
	void EvalDynamicTerms(const tVectorXd& ref_feature, const tVectorXd& ref_tau, tMatrixXd& H, tMatrixXd& E, tVectorXd& f) const;
	tVectorXd CalcTargetFeature(const tVectorXd& qddot);
	int GetSingleFeatureSize(btGenFeature* feature) const;
	int GetSingleFeatureOffset(int feature_id) const;
	btGenFeature* CreateJointFeature(const std::string& joint_name) const;
	btGenFeature* CreateHeightFeature(const std::string& link_name) const;
	btGenFeature* CreateLocationFeature(const std::string& link_name) const;
	void PrintFeatureInfo() const;

	void TestdJvdq(tVectorXd& q, tVectorXd& qdot, tVectorXd& qddot, int link_id);
};