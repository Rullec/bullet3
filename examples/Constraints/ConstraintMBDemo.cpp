#include "ConstraintMBDemo.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h>  //printf debugging
#include <cmath>
#include <iostream>
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "ConstraintMultibody.hpp"

// ConstraintMBDemo shows how to construct multibody system 
// by adding hinge/spherical constraint between objects
class ConstraintMBDemo : public CommonRigidBodyBase
{
	//keep track of variables to delete memory at the end

	void setupEmptyDynamicsWorld();

public:
	ConstraintMBDemo(struct GUIHelperInterface* helper);

	virtual ~ConstraintMBDemo();

	virtual void initPhysics();

	virtual void exitPhysics();

	virtual void stepSimulation(float deltaTime) override;

	virtual void resetCamera()
	{
		float dist = 27;
		float pitch = -30;
		float yaw = 720;
		float targetPos[3] = {2, 0, -10};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	virtual bool keyboardCallback(int key, int state);

	// for cone-twist motor driving
	float m_Time;
	int mFrame;
	// class btConeTwistConstraint* m_ctc;
	cConstraintMultibody * mMultibody;

	struct RecordInfo{
		btVector3 LinearMomentumPre, LinearMomentumPost,
				AngMomentumPre, AngMomentumPost;
		btVector3 TotalExtForce, TotalExtTorque;
	} mRecord;
};

#define ENABLE_ALL_DEMOS 1

#define CUBE_HALF_EXTENTS 1.f

#define SIMD_PI_2 ((SIMD_PI)*0.5f)
#define SIMD_PI_4 ((SIMD_PI)*0.25f)

btTransform sliderTransform_self;
btVector3 lowerSliderLimit_self = btVector3(-10, 0, 0);
btVector3 hiSliderLimit_self = btVector3(10, 0, 0);

btRigidBody* d6body0_self = 0;

btHingeConstraint* spDoorHinge_self = NULL;
btHingeConstraint* spHingeDynAB_self = NULL;
btPoint2PointConstraint * p2pCons_self = NULL;
btGeneric6DofConstraint* spSlider6Dof_self = NULL;

static bool s_bTestConeTwistMotor_self = false;

void ConstraintMBDemo::setupEmptyDynamicsWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	m_solver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
}

void ConstraintMBDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	m_Time = 0;
	mFrame = 0; 

	setupEmptyDynamicsWorld();

	m_dynamicsWorld->setGravity(btVector3(0, 0, 0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(40.),btScalar(50.)));
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 40);

	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -56, 0));
	btRigidBody* groundBody;
	groundBody = createRigidBody(0, groundTransform, groundShape);

	// create multibody
	int numLinks = 3;
	bool floatingBase = false;
	bool spherical = true;
	mMultibody = new cConstraintMultibody(m_dynamicsWorld,m_collisionShapes,  numLinks, floatingBase, spherical);

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void ConstraintMBDemo::exitPhysics()
{
	int i;

	//removed/delete constraints
	for (i = m_dynamicsWorld->getNumConstraints() - 1; i >= 0; i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}

	//remove the rigidbodies from the dynamics world and delete them
	for (i = m_dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < m_collisionShapes.size(); j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld = 0;

	//delete solver
	delete m_solver;
	m_solver = 0;

	//delete broadphase
	delete m_broadphase;
	m_broadphase = 0;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void ConstraintMBDemo::stepSimulation(float deltaTime)
{
	assert(mMultibody);
	deltaTime = 3e-3;
	m_Time += deltaTime;
	std::cout <<"[log] ConstraintMBDemo::stepSimulation " << deltaTime <<", cur time " << m_Time <<", frame num = " << mFrame << std::endl;

	
	if(m_dynamicsWorld)
	{
		mRecord.LinearMomentumPre = mMultibody->CalcLinearMomentum();
		mRecord.AngMomentumPre = mMultibody->CalcAngMomentum();
		mRecord.TotalExtForce = mMultibody->GetTotalMass() * m_dynamicsWorld->getGravity();
		// exit(1);
		m_dynamicsWorld->stepSimulation(deltaTime, 0, deltaTime);
		mRecord.AngMomentumPost = mMultibody->CalcAngMomentum();
		mRecord.LinearMomentumPost = mMultibody->CalcLinearMomentum();
		btVector3 lin_diff = (mRecord.LinearMomentumPost - mRecord.LinearMomentumPre) - mRecord.TotalExtForce * deltaTime;
		btVector3 ang_diff = (mRecord.AngMomentumPost - mRecord.AngMomentumPre);
		std::cout <<"linear mom diff= " << btBulletUtil::btVectorTotVector0(lin_diff).transpose() << std::endl;
		std::cout <<"ang mom diff= " << btBulletUtil::btVectorTotVector0(ang_diff).transpose() << std::endl;
		std::cout <<"ang mom = " << btBulletUtil::btVectorTotVector0(mRecord.AngMomentumPost).transpose() << std::endl;
	}
	// std::cout <<"pos = " << btBulletUtil::btVectorTotVector0(mMultibody->GetLinkWorldPos(0)).transpose() << std::endl;
	// std::cout <<"vel = " << btBulletUtil::btVectorTotVector0(mMultibody->GetLinkWorldVel(0)).transpose() << std::endl;
	
	
	mFrame++;
}

ConstraintMBDemo::ConstraintMBDemo(struct GUIHelperInterface* helper)
	: CommonRigidBodyBase(helper)
{
}

ConstraintMBDemo::~ConstraintMBDemo()
{
	//cleanup in the reverse order of creation/initialization

	btAssert(m_dynamicsWorld == 0);
}

bool ConstraintMBDemo::keyboardCallback(int key, int state)
{
	bool handled = false;

	switch (key)
	{
		case 'O':
		{
			bool offectOnOff;
			if (spDoorHinge_self)
			{
				offectOnOff = spDoorHinge_self->getUseFrameOffset();
				offectOnOff = !offectOnOff;
				spDoorHinge_self->setUseFrameOffset(offectOnOff);
				printf("DoorHinge %s frame offset\n", offectOnOff ? "uses" : "does not use");
			}
			if (spHingeDynAB_self)
			{
				offectOnOff = spHingeDynAB_self->getUseFrameOffset();
				offectOnOff = !offectOnOff;
				spHingeDynAB_self->setUseFrameOffset(offectOnOff);
				printf("HingeDynAB %s frame offset\n", offectOnOff ? "uses" : "does not use");
			}
			if (spSlider6Dof_self)
			{
				offectOnOff = spSlider6Dof_self->getUseFrameOffset();
				offectOnOff = !offectOnOff;
				spSlider6Dof_self->setUseFrameOffset(offectOnOff);
				printf("Slider6Dof %s frame offset\n", offectOnOff ? "uses" : "does not use");
			}
		}
			handled = true;
			break;
		default:
		{
		}
		break;
	}
	return handled;
}

class CommonExampleInterface* ConstraintMBDemoCreateFunc(struct CommonExampleOptions& options)
{
	return new ConstraintMBDemo(options.m_guiHelper);
}