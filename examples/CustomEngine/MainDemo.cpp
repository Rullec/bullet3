#include "MainDemo.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Z 1

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "Simulator.h"
#include "json/json.h"
#include <memory>
#include <iostream>

struct CustomEngineMainDemo : public CommonRigidBodyBase
{
	struct tParams
	{
		bool mAddObj;
		bool mAddMultibody;
		bool mEnableGravity;
		bool mEnableLCP;
		double mDamping;
		int mMultiLinkNum;
		int mObjLinkNum;
		tParams(const std::string& path);
	};

	struct tParams* physics_param;
	CustomEngineMainDemo(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
		// mTime = 0;
		physics_param = nullptr;
	}
	virtual ~CustomEngineMainDemo()
	{
	}
	virtual void stepSimulation(float deltaTime) override final;
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 3;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0.46, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

protected:
	cSimulator* mSimulator;
	// btRigidBody* target_rigidbody;
	// float mTime;
};

void CustomEngineMainDemo::stepSimulation(float dt)
{
	// mTime += dt;
	// std::cout << "cur time = " << mTime << std::endl;
	// m_guiHelper;
	mSimulator->StepSimulation(dt);
	// CommonRigidBodyBase::stepSimulation(dt);
	// if (mTime > 1)
	// {
	// 	m_dynamicsWorld->removeCollisionObject(target_rigidbody);
	// 	m_collisionShapes.pop_back();
	// }
	// std::cout <<"collision shapes = " << m_collisionShapes.size() << std::endl;
}

void CustomEngineMainDemo::initPhysics()
{
	physics_param = new tParams("./examples/CustomEngine/config.json");

	m_guiHelper->setUpAxis(1);

	cSimulator::tParams simulator_params;
	if (physics_param->mEnableGravity)
		simulator_params.mGravity = tVector(0, -9.8, 0, 0);
	else
		simulator_params.mGravity = tVector(0, 0, 0, 0);

	simulator_params.damping = physics_param->mDamping;

	if (physics_param->mEnableLCP == true)
		simulator_params.Mode = cSimulator::eContactResponseMode::LCPMode;
	else
		simulator_params.Mode = cSimulator::eContactResponseMode::PenaltyMode;

	mSimulator = new cSimulator(simulator_params);
	mSimulator->Init();
	m_dynamicsWorld = mSimulator->GetInternalWorld();

	if (physics_param->mAddObj)
		mSimulator->AddObj(physics_param->mObjLinkNum);
	if (physics_param->mAddMultibody)
		mSimulator->AddMultibody(physics_param->mMultiLinkNum);

	mSimulator->AddGround();

	// {
	// 	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(1.), btScalar(1.), btScalar(1.)));
	// 	btVector3 localInertia(0, 0, 0);
	// 	groundShape->calculateLocalInertia(0, localInertia);
	// 	btRigidBody* body = new btRigidBody(0, 0, groundShape, localInertia);

	// 	body->setUserIndex(-1);
	// 	m_dynamicsWorld->addRigidBody(body);
	// }

	// {
	// 	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(0.1), btScalar(0.1), btScalar(0.1)));
	// 	btVector3 localInertia(0, 0, 0);
	// 	groundShape->calculateLocalInertia(0, localInertia);
	// 	btCollisionObject * obj = new btCollisionObject();
	// 	obj->setCollisionShape(groundShape);
	// 	// btRigidBody* body = new btRigidBody(0, 0, groundShape, localInertia);

	// 	obj->setUserIndex(-1);
	// 	m_dynamicsWorld->addCollisionObject(obj);
	// }

	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void CustomEngineMainDemo::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

#include <fstream>
CustomEngineMainDemo::tParams::tParams(const std::string& path)
{
	Json::Value json_root;
	Json::CharReaderBuilder builder;
	std::ifstream fin(path, std::ifstream::binary);
	std::string errs;
	bool ok = Json::parseFromStream(builder, fin, &json_root, &errs);
	if (false == ok)
	{
		std::cout << "parse json or open file failed: " << path << std::endl;
		exit(0);
	}
	mAddMultibody = json_root["add_multibody"].asBool();
	mAddObj = json_root["add_obj"].asBool();
	mEnableGravity = json_root["enable_gravity"].asBool();
	mEnableLCP = json_root["enable_lcp"].asBool();
	mDamping = json_root["damping"].asDouble();
	mMultiLinkNum = json_root["multibody_num"].asInt();
	mObjLinkNum = json_root["obj_num"].asInt();
}

CommonExampleInterface* CustomMainCreateFunc(CommonExampleOptions& options)
{
	return new CustomEngineMainDemo(options.m_guiHelper);
}