#pragma once
#include "../ExampleBrowser/ID_test/BulletUtil.h"
#include "btBulletDynamicsCommon.h"
#include <memory>

class cSimRigidBody;
class cContactSolver;
class cRobotModel;
class cSimulator
{
public:
	enum eContactResponseMode
	{
		LCPMode,
		PenaltyMode
	};

	struct tParams
	{
		tParams();
		tVector mGravity;
		double damping;
		eContactResponseMode Mode;
	};

	cSimulator(const tParams& params);
	~cSimulator();
	void Init();
	void AddObj(int n);
	void AddGround();
	void AddMultibody(int n);
	void RemoveObj(int id);
	void StepSimulation(float dt);
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
	cRobotModel* mMultibody;
	float mTime;
	tVector mGravity;
	float mDamping;
	eContactResponseMode mContactMode;
	cContactSolver* mLCPContactSolver;
	std::vector<btPersistentManifold*> mManifolds;

	void createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const std::string& name, const btVector4& color = btVector4(1, 0, 0, 1));
	void createRigidBodyNew(float mass, const btTransform& startTransform, btCollisionShape* shape, const std::string& name, const btVector4& color = btVector4(1, 0, 0, 1));
	btBoxShape* createBoxShape(const btVector3& halfExtents);

	// 1. use active force to update the velocity of the world
	void ApplyActiveForce(float dt);

	// 2. collision detect
	void CollisionDetect();

	// 3. collision respose
	void CollisionRespose(float dt);
	void CollisionResposePenalty(float dt);
	void CollisionResposeLCP(float dt);

	// 4. Clear active forces
	void ClearActiveForce();

	// 5. update transforms
	void UpdateTransform(float dt);

	void UpdateVelocity(float dt);
	void PostUpdate(float dt);
};