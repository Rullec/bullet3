#pragma once
#include "../ExampleBrowser/ID_test/BulletUtil.h"
#include "btBulletDynamicsCommon.h"
#include <memory>

class cSimRigidBody;
class cContactSolver;
class cRobotModelDynamics;
class tContactForce;
class tConstraintGeneralizedForce;
class cSimulator
{
public:
	enum eContactResponseMode
	{
		LCPMode,
		PenaltyMode,
		SequentialImpulseMode
	};

	// struct tParams
	// {
	// 	tParams();
	// 	std::string mSimulatorConfig;
	// tVector mGravity;
	// double rigid_damping;
	// double mb_damping1;
	// double mb_damping2;
	// double mb_scale;
	// bool mb_enable_rk4;
	// eContactResponseMode Mode;
	// std::string mLCPConfigPath;
	// };

	// cSimulator(const tParams& params);
	cSimulator(const std::string& config_path);
	~cSimulator();
	void Init();
	void AddObj(int n, const std::string& obj_type, bool perturb = false);
	void AddGround();
	void AddMultibody(const std::string& path);
	void RemoveObj(int id);
	void StepSimulation(double dt);
	void GetContactInfo();

	// get & set method
	btDiscreteDynamicsWorld* GetInternalWorld();
	btBroadphaseInterface* GetBroadphase();
	btCollisionDispatcher* GetDispatcher();
	btDefaultCollisionConfiguration* GetConfiguration();

protected:
	btDiscreteDynamicsWorld* mInternalWorld;
	btBroadphaseInterface* m_broadphase;
	btCollisionDispatcher* m_dispatcher;
	btDefaultCollisionConfiguration* m_collisionConfiguration;
	std::vector<cSimRigidBody*> mSimObjs;
	cRobotModelDynamics* mMultibody;
	double mTime;
	tVector mGravity;
	double mRigidDamping;
	double mMBDamping1;
	double mMBDamping2;
	bool mMBZeroInitPose;
	bool mMBZeroInitPoseVel;
	bool mMBEnableRk4;
	double mMBEpsDiagnoalMassMat;
	double mMBMaxVel;
	bool mMBUpdateVelWithoutCoriolis;
	bool mEnablePeturb;
	bool mMBEAngleClamp;
	bool mMBTestJacobian;
	bool mMBEnableCollectFrameInfo;
	bool mEnablePauseWhenMaxVel;
	int mMBCollectFrameNum;
	double mMBScale;
	eContactResponseMode mContactMode;
	cContactSolver* mLCPContactSolver;
	std::string mLCPConfigPath;
	std::vector<btPersistentManifold*> mManifolds;
	std::vector<tContactForce*> mContactForces;
	std::vector<tConstraintGeneralizedForce*> mConstraintGenalizedForce;

	void createRigidBody(double mass, const btTransform& startTransform, btCollisionShape* shape, const std::string& name, const btVector4& color = btVector4(1, 0, 0, 1));
	void createRigidBodyNew(double mass, const btTransform& startTransform, btCollisionShape* shape, const std::string& name, const btVector4& color = btVector4(1, 0, 0, 1));
	btBoxShape* createBoxShape(const btVector3& halfExtents);

	// 1. use active force to update the velocity of the world
	void ApplyActiveForce(double dt);

	// 2. collision detect
	void CollisionDetect();

	// 3. collision respose
	void CollisionRespose(double dt);
	void CollisionResposePenalty(double dt);
	void CollisionResposeLCP(double dt);
	void CollisionResposeSI(double dt);
	void PushStatePreCollision();
	void PopStatePostColliison();

	// 4. Clear active forces
	void ClearForce();

	void Update(double dt);
	// void UpdateTransform(double dt);
	void UpdateVelocityInternal(double dt);
	void UpdateVelocityInternalWithoutCoriolis(double dt);
	void PostUpdate(double dt);

	// sim record
	struct tFrameInfo
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		int frame_id;
		double timestep;
		tVectorXd q, qdot, qddot;  // generalized displacement, generalized vel, gen accel
		tVectorXd Q, residual;     //  gen force, lagrangian residual
		tMatrixXd mass_mat, coriolis_mat, damping_mat, mass_mat_inv;

		tEigenArr<tVector> force_array, torque_array;
	};
	tEigenArr<tFrameInfo> mFrameInfo;
	void CollectFrameInfo(double dt);
	void WriteFrameInfo(const std::string& path);
};