// #pragma once
#include "MultiDofDemo.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include "btBulletDynamicsCommon.h"

#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"

#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodySliderConstraint.h"

#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "ID_test/cIDSolver.h" 
#include "BulletDynamics/Featherstone/cCollisionWorld.h"

#include <fstream>
#include <iostream>

#define OUTPUT_SIMULATION_LOG

class MultiDofDemo : public CommonMultiBodyBase
{
public:
	MultiDofDemo(GUIHelperInterface* helper);
	virtual ~MultiDofDemo();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	virtual void resetCamera()
	{
		float dist = 1;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = {-3, 2.8, -2.5};
		//float targetPos[3] = {-0.4, 0.4, -0.4};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	btMultiBody* createFeatherstoneMultiBody_testMultiDof(class btMultiBodyDynamicsWorld* world, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical = false, bool floating = false);
	void addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents);
	void addBoxes_testMultiDof();

protected:
	btMultiBody* mMultibody;
	cIDSolver * mIDSolver;
	int mFrameId;

	// log path
	std::ofstream mSimLog;

	// ID tool methods
	void RecordSimLog();
};

static bool g_floatingBase = true;
static bool g_firstInit = true;
static float scaling = 0.4f;
static float friction = 1.;
static int g_constraintSolverType = 0;
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z + 1024)

#define START_POS_X -5
//#define START_POS_Y 12
#define START_POS_Y 2
#define START_POS_Z -3

MultiDofDemo::MultiDofDemo(GUIHelperInterface* helper) : CommonMultiBodyBase(helper)
{
	m_guiHelper->setUpAxis(1);

	// init vars
	mMultibody = nullptr;

	// rewrite all log files to null
	mSimLog.open("custom_world_sim.log");
	
	mSimLog << "";
	mFrameId = 0;

}
MultiDofDemo::~MultiDofDemo()
{
	mSimLog.close();
}


btVector3 gGravity = btVector3(0, -10, 0);

void MultiDofDemo::stepSimulation(float deltaTime)
{
	deltaTime = 3e-3;
#ifdef OUTPUT_SIMULATION_LOG
	RecordSimLog();
#endif // OUTPUT_SIMULATION_LOG

	mFrameId++;
	
	mIDSolver->SetTimestep(deltaTime);
	mIDSolver->PreSim();

// for easy debug 
	m_dynamicsWorld->stepSimulation(deltaTime, 0, deltaTime);

	mIDSolver->PostSim();
}

class CommonExampleInterface* MultiDofCreateFunc(struct CommonExampleOptions& options)
{
	return new MultiDofDemo(options.m_guiHelper);
}


void MultiDofDemo::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	if (g_firstInit)
	{
		m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraDistance(btScalar(10. * scaling));
		m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraPitch(50);
		g_firstInit = false;
	}
	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	if (g_constraintSolverType == 4)
	{
		g_constraintSolverType = 0;
		g_floatingBase = !g_floatingBase;
	}

	btMultiBodyConstraintSolver* sol;
	btMLCPSolverInterface* mlcp;
	switch (g_constraintSolverType++)
	{
	case 0:
		sol = new btMultiBodyConstraintSolver;
		b3Printf("Constraint Solver: Sequential Impulse");
		break;
	case 1:
		mlcp = new btSolveProjectedGaussSeidel();
		sol = new btMultiBodyMLCPConstraintSolver(mlcp);
		b3Printf("Constraint Solver: MLCP + PGS");
		break;
	case 2:
		mlcp = new btDantzigSolver();
		sol = new btMultiBodyMLCPConstraintSolver(mlcp);
		b3Printf("Constraint Solver: MLCP + Dantzig");
		break;
	default:
		mlcp = new btLemkeSolver();
		sol = new btMultiBodyMLCPConstraintSolver(mlcp);
		b3Printf("Constraint Solver: MLCP + Lemke");
		break;
	}

	m_solver = sol;

	// create new world
	{
		cCollisionWorld * my_world = new cCollisionWorld(m_dispatcher, m_broadphase, sol, m_collisionConfiguration);
		btMultiBodyDynamicsWorld * world = static_cast<btMultiBodyDynamicsWorld *>(my_world);
		if (world != nullptr && nullptr != world)
		{
			std::cout << "[log] create cCollisionWorld succ\n";
		}
		m_dynamicsWorld = world;
		m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
		m_dynamicsWorld->setGravity(gGravity);
		m_dynamicsWorld->getSolverInfo().m_globalCfm = 1e-3;
	}




	///create a few basic rigid bodies
	btVector3 groundHalfExtents(50, 50, 50);
	btCollisionShape* groundShape = new btBoxShape(groundHalfExtents);
	//groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;

	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////
	bool damping = false;
	bool gyro = true;
	int numLinks = 2;
	bool spherical = false;  //set it ot false -to use 1DoF hinges instead of 3DoF sphericals
	bool multibodyOnly = false;
	bool canSleep = false;
	bool selfCollide = true;
	bool multibodyConstraint = false;
	btVector3 linkHalfExtents(0.05, 0.4, 0.1);
	btVector3 baseHalfExtents(0.05, 0.4, 0.1);

	btMultiBody* mbC = createFeatherstoneMultiBody_testMultiDof(m_dynamicsWorld, numLinks, btVector3(0.f, 0.f, 0.f), linkHalfExtents, baseHalfExtents, spherical, g_floatingBase);
	//mbC->forceMultiDof();							//if !spherical, you can comment this line to check the 1DoF algorithm
	mMultibody = mbC;

	mbC->setCanSleep(canSleep);
	mbC->setHasSelfCollision(selfCollide);
	mbC->setUseGyroTerm(gyro);
	//
	if (!damping)
	{
		mbC->setLinearDamping(0.f);
		mbC->setAngularDamping(0.f);
	}
	else
	{
		mbC->setLinearDamping(0.1f);
		mbC->setAngularDamping(0.9f);
	}
	//
	mbC->setBaseVel(btVector3(10, 10, 10));
	mbC->setBaseOmega(btVector3(10, 10, 10));
	// mbC->setMaxCoordinateVelocity(3.0f);
	// mbC->setBaseOmega(btVector3(0, 0, 5));
	m_dynamicsWorld->setGravity(gGravity);
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
	//////////////////////////////////////////////
	if (numLinks > 0)
	{
		btScalar q0 = 90.f * SIMD_PI / 180.f;
		//btScalar q0 = 90.f * SIMD_PI / 180.f;
		if (!spherical)
		{
			mbC->setJointPosMultiDof(0, &q0);
		}
		else
		{
			btQuaternion quat0(btVector3(1, 1, 0).normalized(), q0);
			quat0.normalize();
			mbC->setJointPosMultiDof(0, quat0);
		}
	}
	///
	addColliders_testMultiDof(mbC, m_dynamicsWorld, baseHalfExtents, linkHalfExtents);

	/////////////////////////////////////////////////////////////////
	btScalar groundHeight = -52.55;
	if (!multibodyOnly)
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, groundHeight, 0));
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body, 1, 1 + 2);  //,1,1+2);
	}
	/////////////////////////////////////////////////////////////////
	//if (!multibodyOnly)
	//{
	//	btVector3 halfExtents(.5, .5, .5);
	//	btBoxShape* colShape = new btBoxShape(halfExtents);
	//	//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	//	m_collisionShapes.push_back(colShape);
	//	/// Create Dynamic Objects
	//	btTransform startTransform;
	//	startTransform.setIdentity();
	//	btScalar mass(1.f);
	//	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	//	bool isDynamic = (mass != 0.f);
	//	btVector3 localInertia(0, 0, 0);
	//	if (isDynamic)
	//		colShape->calculateLocalInertia(mass, localInertia);
	//	startTransform.setOrigin(btVector3(
	//		btScalar(0.0),
	//		0.0,
	//		btScalar(0.0)));
	//	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	//	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	//	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	//	btRigidBody* body = new btRigidBody(rbInfo);
	//	m_dynamicsWorld->addRigidBody(body);  //,1,1+2);
	//	if (multibodyConstraint)
	//	{
	//		btVector3 pointInA = -linkHalfExtents;
	//		//      btVector3 pointInB = halfExtents;
	//		btMatrix3x3 frameInA;
	//		btMatrix3x3 frameInB;
	//		frameInA.setIdentity();
	//		frameInB.setIdentity();
	//		btVector3 jointAxis(1.0, 0.0, 0.0);
	//		//btMultiBodySliderConstraint* p2p = new btMultiBodySliderConstraint(mbC,numLinks-1,body,pointInA,pointInB,frameInA,frameInB,jointAxis);
	//		btMultiBodyFixedConstraint* p2p = new btMultiBodyFixedConstraint(mbC, numLinks - 1, mbC, numLinks - 4, pointInA, pointInA, frameInA, frameInB);
	//		p2p->setMaxAppliedImpulse(2.0);
	//		m_dynamicsWorld->addMultiBodyConstraint(p2p);
	//	}
	//}

	// init Inverse Model
	mIDSolver = new cIDSolver(mMultibody, m_dynamicsWorld);
	mIDSolver->ClearID();

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	/////////////////////////////////////////////////////////////////
}

btMultiBody* MultiDofDemo::createFeatherstoneMultiBody_testMultiDof(btMultiBodyDynamicsWorld* pWorld, int numLinks, const btVector3& basePosition, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents, bool spherical, bool floating)
{
	//init the base
	btVector3 baseInertiaDiag(0.f, 0.f, 0.f);
	float baseMass = 1.f;

	if (baseMass)
	{
		btCollisionShape* pTempBox = new btBoxShape(btVector3(baseHalfExtents[0], baseHalfExtents[1], baseHalfExtents[2]));
		pTempBox->calculateLocalInertia(baseMass, baseInertiaDiag);
		delete pTempBox;
	}

	bool canSleep = false;
	btMultiBody* pMultiBody = new btMultiBody(numLinks, baseMass, baseInertiaDiag, !floating, canSleep);
	

	// 这里设置的是world to base rot，也就是一个世界坐标系向量想要转换到base坐标系中，需要旋转多少角度。
	// 坐标系的旋转和向量的旋转是相反角度关系。
	double theta = 0.0 / 180 * M_PI;
	double cos_theta_2 = std::cos(theta / 2),
			sin_theta_2 = std::sin(theta / 2);
	//btQuaternion baseOriQuat(0.f, 0.5f, 0.f, 0.866f);
	// btQuaternion baseOriQuat(0.f, 0.f, 0.f, 1.f);
	btQuaternion baseOriQuat(1.0f * sin_theta_2, 0.f, 0.f, 1.f * cos_theta_2);

	pMultiBody->setBasePos(basePosition);
	pMultiBody->setWorldToBaseRot(baseOriQuat);
	btVector3 vel(0, 0, 0);

	btTransform base_world_trans;
	btQuaternion world_to_base_rot, base_world_rot;
	base_world_trans = pMultiBody->getBaseWorldTransform();
	world_to_base_rot = pMultiBody->getWorldToBaseRot();
	base_world_rot = base_world_trans.getRotation();
	std::cout <<"raw world to base rot = " << cBulletUtil::btQuaternionTotQuaternion(baseOriQuat).coeffs().transpose() << std::endl;
	std::cout <<"[log] world_to_base_rot = " << cBulletUtil::btQuaternionTotQuaternion(world_to_base_rot).coeffs().transpose() << std::endl;
	std::cout <<"[log] base_world_rot = " << cBulletUtil::btQuaternionTotQuaternion(base_world_rot).coeffs().transpose() << std::endl;

	//init the links
	btVector3 hingeJointAxis(1, 0, 0);
	float linkMass = 1.f;
	btVector3 linkInertiaDiag(0.f, 0.f, 0.f);

	btCollisionShape* pTempBox = new btBoxShape(btVector3(linkHalfExtents[0], linkHalfExtents[1], linkHalfExtents[2]));
	pTempBox->calculateLocalInertia(linkMass, linkInertiaDiag);
	delete pTempBox;

	//y-axis assumed up
	btVector3 parentComToCurrentCom(0, -linkHalfExtents[1] * 2.f, 0);                      //par body's COM to cur body's COM offset
	btVector3 currentPivotToCurrentCom(0, -linkHalfExtents[1], 0);                         //cur body's COM to cur body's PIV offset
	btVector3 parentComToCurrentPivot = parentComToCurrentCom - currentPivotToCurrentCom;  //par body's COM to cur body's PIV offset

	//////
	//btScalar q0 = 0.f * SIMD_PI / 180.f;
	//btQuaternion quat0(btVector3(1, 1, 1).normalized(), q0);
	//quat0.normalize();
	/////

	for (int i = 0; i < numLinks; ++i)
	{
		if (!spherical)
			pMultiBody->setupRevolute(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), hingeJointAxis, parentComToCurrentPivot, currentPivotToCurrentCom, true);
		else
			//pMultiBody->setupPlanar(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f)/*quat0*/, btVector3(1, 0, 0), parentComToCurrentPivot*2, false);
			pMultiBody->setupSpherical(i, linkMass, linkInertiaDiag, i - 1, btQuaternion(0.f, 0.f, 0.f, 1.f), parentComToCurrentPivot, currentPivotToCurrentCom, true);
	}

	pMultiBody->finalizeMultiDof();

	///
	pWorld->addMultiBody(pMultiBody);
	///
	return pMultiBody;
}

void MultiDofDemo::addColliders_testMultiDof(btMultiBody* pMultiBody, btMultiBodyDynamicsWorld* pWorld, const btVector3& baseHalfExtents, const btVector3& linkHalfExtents)
{
	btAlignedObjectArray<btQuaternion> world_to_local;
	world_to_local.resize(pMultiBody->getNumLinks() + 1);

	btAlignedObjectArray<btVector3> local_origin;
	local_origin.resize(pMultiBody->getNumLinks() + 1);
	world_to_local[0] = pMultiBody->getWorldToBaseRot();
	local_origin[0] = pMultiBody->getBasePos();

	{
		//	float pos[4]={local_origin[0].x(),local_origin[0].y(),local_origin[0].z(),1};
		btScalar quat[4] = { -world_to_local[0].x(), -world_to_local[0].y(), -world_to_local[0].z(), world_to_local[0].w() };

		if (1)
		{
			btCollisionShape* box = new btBoxShape(baseHalfExtents);
			btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, -1);
			col->setCollisionShape(box);

			btTransform tr;
			tr.setIdentity();
			tr.setOrigin(local_origin[0]);
			tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
			col->setWorldTransform(tr);

			pWorld->addCollisionObject(col, 2, 1 + 2);

			col->setFriction(friction);
			pMultiBody->setBaseCollider(col);
		}
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		const int parent = pMultiBody->getParent(i);
		world_to_local[i + 1] = pMultiBody->getParentToLocalRot(i) * world_to_local[parent + 1];
		local_origin[i + 1] = local_origin[parent + 1] + (quatRotate(world_to_local[i + 1].inverse(), pMultiBody->getRVector(i)));
	}

	for (int i = 0; i < pMultiBody->getNumLinks(); ++i)
	{
		btVector3 posr = local_origin[i + 1];
		//	float pos[4]={posr.x(),posr.y(),posr.z(),1};

		btScalar quat[4] = { -world_to_local[i + 1].x(), -world_to_local[i + 1].y(), -world_to_local[i + 1].z(), world_to_local[i + 1].w() };

		btCollisionShape* box = new btBoxShape(linkHalfExtents);
		btMultiBodyLinkCollider* col = new btMultiBodyLinkCollider(pMultiBody, i);

		col->setCollisionShape(box);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(posr);
		tr.setRotation(btQuaternion(quat[0], quat[1], quat[2], quat[3]));
		col->setWorldTransform(tr);
		col->setFriction(friction);
		pWorld->addCollisionObject(col, 2, 1 + 2);

		pMultiBody->getLink(i).m_collider = col;
	}
}

void MultiDofDemo::addBoxes_testMultiDof()
{
	//create a few dynamic rigidbodies
	// Re-using the same collision is better for memory usage and performance

	btBoxShape* colShape = new btBoxShape(btVector3(1, 1, 1));
	//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
	m_collisionShapes.push_back(colShape);

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();

	btScalar mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	float start_x = START_POS_X - ARRAY_SIZE_X / 2;
	float start_y = START_POS_Y;
	float start_z = START_POS_Z - ARRAY_SIZE_Z / 2;

	for (int k = 0; k < ARRAY_SIZE_Y; k++)
	{
		for (int i = 0; i < ARRAY_SIZE_X; i++)
		{
			for (int j = 0; j < ARRAY_SIZE_Z; j++)
			{
				startTransform.setOrigin(btVector3(
					btScalar(3.0 * i + start_x),
					btScalar(3.0 * k + start_y),
					btScalar(3.0 * j + start_z)));

				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);

				m_dynamicsWorld->addRigidBody(body);  //,1,1+2);
			}
		}
	}
}

void MultiDofDemo::RecordSimLog()
{
	// output the simulation log
	mSimLog << "-----------------frame " << mFrameId << "-----------------\n";
	btTransform trans;

	for (int i = 0; i < mMultibody->getNumLinks() + 1; i++)
	{
		if (i == 0)
		{
			mSimLog << "for root link:\n";
			mSimLog << "\tpos = " << cBulletUtil::btVectorTotVector1(mMultibody->getBasePos()).transpose() << std::endl;;
			mSimLog << "\torientation = " << cBulletUtil::btQuaternionTotQuaternion(mMultibody->getWorldToBaseRot().inverse()).coeffs().transpose() << std::endl;
		}
		else
		{
			int multibody_id = i - 1;
			trans = mMultibody->getLinkCollider(multibody_id)->getWorldTransform();
			mSimLog << "for link: " << multibody_id << "\n";
			mSimLog << "\tpos = " << cBulletUtil::btVectorTotVector1(trans.getOrigin()).transpose() << std::endl;;
			mSimLog << "\torientation = " << cBulletUtil::btQuaternionTotQuaternion(trans.getRotation()).coeffs().transpose() << std::endl;;

		}
	}
}