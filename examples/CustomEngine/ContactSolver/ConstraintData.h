#include "../../ExampleBrowser/ID_test/MathUtil.h"
#include "../ColObjBase.h"

class cSimRigidBody;
struct cRobotCollider;

struct tContactPointData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	tContactPointData(int c_id, double dt);
	int contact_id;
	double dt;  // timestep
	cCollisionObject *mBodyA, *mBodyB;
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

	void TestHt(int n);
	void Setup(int num_frictions);
	void ApplyForceResultVector(const tVectorXd& x0);
	void ApplyForceCartersian(const tVector& force);

	bool CheckOverlap(tContactPointData* other_data);
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
struct tCollisionObjData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	tCollisionObjData(cCollisionObject*);
	void AddContactPoint(tContactPointData*, bool isbody0);
	void Clear();
	void Setup(int n_total_contacts);

	cCollisionObject* mBody;
	tMatrixXd mConvertCartesianForceToVelocityMat;
	tVectorXd mConvertCartesianForceToVelocityVec;

protected:
	std::vector<tContactPointData*> mContactPts;
	std::vector<bool> mIsBody0;
	void SetupRobotCollider();
	void SetupRigidBody();
};