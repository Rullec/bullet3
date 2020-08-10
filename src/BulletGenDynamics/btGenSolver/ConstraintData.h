#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include "../btGenModel/ColObjBase.h"

class btGenRigidBody;
struct btGenRobotCollider;
class cRobotModelDynamics;
#include <map>
struct btGenJointLimitData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	btGenJointLimitData(int constraint_id, cRobotModelDynamics* multibody,
					int dof_id, bool is_upper_bound);

	void ApplyGeneralizedForce(double val);
	void ApplyJointTorque(double val);
	int constraint_id;
	cRobotModelDynamics* multibody;
	double dt;
	int dof_id;
	bool is_upper_bound;
	tVector3d joint_direction;
};

struct btGenContactPointData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	btGenContactPointData(int c_id, double dt);
	int contact_id;
	double dt;  // timestep
	btGenCollisionObject *mBodyA, *mBodyB;
	eColObjType mTypeA, mTypeB;
	tVector mNormalPointToA;
	tVector mContactPtOnA, mContactPtOnB;
	double distance;
	bool mIsSelfCollision;

	int mBodyId0, mBodyId1;
	int mbody0GroupId, mbody1GroupId;
	tMatrixXd mD;        // friction cone matrix 3xN, N is num of frictins
	tMatrixXd mS;        // convert matrix from x0 \in R^{N+2} to f0 \in R^{3}, S.shape = (3, N+2)
	int mNumOfFriction;  //friction number

	double mSI_normal_contact_value;      // total normal contact magnitute
	tVector mSI_tangent_contact_dir[2];   // 2 tangent friction directions
	double mSI_tangent_contact_value[2];  // tangent friction magnitutes on 2 directions

	// u_rel = Z * f + h
	tMatrix mSI_Z;  // mat H convert the cartesian force to relative velocity
	tVector mSI_h;  // vec h convert force to relative vel

	void TestHt(int n);
	void Setup(int num_frictions);
	void ApplyForceResultVector(const tVectorXd& x0);
	void ApplyForceCartersian(const tVector& force);

	bool CheckOverlap(btGenContactPointData* other_data);
	tMatrixXd GetJacobian(int world_col_id);
	tMatrixXd GetConvertMatS(int world_col_id);
	int GetBody0Id();
	int GetBody1Id();
	tVector GetRelVel();
	tVector GetVelOnBody0();
	tVector GetVelOnBody1();
	void UpdateVel(double dt);

protected:
	void CalcBodyInfo();

	void CalcFrictionCone(tMatrixXd& D);
	void CalcConvertMat(tMatrixXd& S);
};

// each collision object will has a data structure
struct btGenCollisionObjData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	btGenCollisionObjData(btGenCollisionObject*);
	void AddContactPoint(btGenContactPointData*, bool isbody0);
	void AddJointLimitConstraint(btGenJointLimitData*);
	void Clear();
	void Setup(int n_total_contacts, int n_total_jointlimits);

	btGenCollisionObject* mBody;
	tMatrixXd mConvertCartesianForceToVelocityMat;
	tVectorXd mConvertCartesianForceToVelocityVec;

	tMatrixXd mSI_ConvertCartesianForceToVelocityMat;
	tVectorXd mSI_ConvertCartesianForceToVelocityVec;

protected:
	std::vector<btGenJointLimitData*> mJointLimits;
	std::vector<btGenContactPointData*> mContactPts;
	std::vector<bool> mIsBody0;
	int num_total_contacts;
	int num_total_joint_limits;
	int num_total_constraints;
	void SetupRobotCollider();
	void SetupRigidBody();
	void SetupRobotColliderVars();
	void GetContactJacobianArrayRobotCollider(tEigenArr<tMatrixXd>& nonself_contact_jac_list, tEigenArr<tMatrixXd>& selfcontact_jac_list);
	void GetJointLimitJaocibanArrayRobotCollider(tEigenArr<tMatrixXd>& jac_list);

	// buffer for multibody
	tMatrixXd M, inv_M, coriolis_mat, damping_mat;

	// vars for multibody
	int num_local_contacts;
	int num_self_contacts;
	int num_nonself_contacts;
	int num_joint_constraint;
	std::map<int, int> map_world_contact_id_to_local_contact_id;    // map the global contact id to the index in "mContactPts" aka local contact id
	std::map<int, int> map_local_contact_id_to_self_contact_id;     // map the local contact id to the index of self contact id
	std::map<int, int> map_local_contact_id_to_nonself_contact_id;  // map the local contact id to the index of nonself contact id
};