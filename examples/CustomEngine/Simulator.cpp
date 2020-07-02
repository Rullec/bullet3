#include "Simulator.h"
#include "SimObj.h"
#include "ContactSolver/ContactSolver.h"
#include "Model/RobotModel.h"
#include <iostream>
#include <fstream>
// #define __DEBUG__

std::map<int, std::string> col_name;
// const std::string path = "momentum.txt";
bool enable_bullet_sim = false;
cSimulator::tParams::tParams()
{
	mGravity = tVector(0, -9.8, 0, 0);
	damping = 0;
	Mode = eContactResponseMode::LCPMode;
}

cSimulator::cSimulator(const tParams& params)
{
	mInternalWorld = nullptr;
	m_broadphase = nullptr;
	m_dispatcher = nullptr;
	m_collisionConfiguration = nullptr;
	mSimObjs.clear();
	mMultibody = nullptr;
	mTime = 0;
	mGravity = params.mGravity;
	mDamping = params.damping;
	mContactMode = params.Mode;
	mLCPContactSolver = nullptr;
	mManifolds.clear();
	// std::ofstream fout(path);
	// fout << "";
	// fout.close();
}

cSimulator::~cSimulator()
{
	std::cout << "Begin simulator deconstrutor\n";
	if (mInternalWorld)
		for (int i = 0; i < mSimObjs.size(); i++) RemoveObj(i);

	delete m_broadphase;
	delete m_dispatcher;
	delete m_collisionConfiguration;
	for (auto& m : mManifolds) delete m;
	delete mMultibody;
	delete mLCPContactSolver;
}

void cSimulator::Init()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	btConstraintSolver* solver = nullptr;
	if (enable_bullet_sim == true)
	{
		solver = new btSequentialImpulseConstraintSolver();
	}

	mInternalWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, solver, m_collisionConfiguration);

	if (mContactMode == LCPMode)
	{
		cContactSolver::tParams params;
		params.mNumFrictionConeDirs = 4;
		params.mWorld = mInternalWorld;
		params.mMu = 0.5;
		mLCPContactSolver = new cContactSolver(params);
	}
}

void cSimulator::AddGround()
{
	// btCollisionShape* childShape = new btSphereShape(btScalar(5));
	// btCompoundShape* colShape = new btCompoundShape();
	// colShape->addChildShape(btTransform::getIdentity(), childShape);
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0, -52, 0));

	// startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI / 6.));
	createRigidBody(0.f, startTransform, groundShape, "ground");
}

void cSimulator::AddObj(int n)
{
	btCollisionShape* childShape = new btSphereShape(btScalar(0.2));
	btCompoundShape* colShape = new btCompoundShape();
	colShape->addChildShape(btTransform::getIdentity(), childShape);
	// btCollisionShape* colShape = new btBoxShape(btVector3(0.2, 0.2, 0.2));

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0, -1, 0));
	// startTransform.setRotation(btQuaternion(btVector3(1, 1, 0), SIMD_PI / 6.));

	// for (int i = 0; i < n; i++)
	// {
	// 	startTransform.setOrigin(startTransform.getOrigin() + btVector3(0, 1.5, 0));
	// 	createRigidBody(1.f, startTransform, colShape, "ball" + std::to_string(i));
	// }

	for (int k = 0; k < n; k++)
	{
		for (int i = 0; i < 1; i++)
		{
			for (int j = 0; j < 1; j++)
			{
				startTransform.setOrigin(btVector3(
					btScalar(1 * i),
					btScalar(2 + 1 * k),
					btScalar(1 * j)));
				startTransform.setOrigin(startTransform.getOrigin() + cBulletUtil::tVectorTobtVector(tVector::Random() * 0.1));
				createRigidBody(1.f, startTransform, colShape, "ball" + std::to_string(k));
			}
		}
	}

	// mSimObjs[1]->SetLinVel(tVector(0, 0, 0, 0));
}

void cSimulator::AddMultibody(int num)
{
	if (mMultibody != nullptr) delete mMultibody;

	const std::string prefix = "../DeepMimic/data/0424/characters/";
	std::string skeleton_name = prefix;
	switch (num)
	{
		case 0:
			skeleton_name += "skeleton_box.json";
			break;
		case 1:
			skeleton_name += "skeleton_042302_revised_0_obj.json";
			break;
		case 2:
			skeleton_name += "skeleton_042302_revised_1_obj.json";
			break;
		case 3:
			skeleton_name += "skeleton_042302_revised_2_obj.json";
			break;
		case 4:
			skeleton_name += "skeleton_042302_revised_leftleg.json";
			break;
		case 10:
			skeleton_name += "skeleton_042302_revised.json";
			break;
		default:
			skeleton_name += "skeleton_sphere.json";
			break;
	}
	mMultibody = new cRobotModel(skeleton_name.c_str(), ModelType::JSON);
	mMultibody->InitSimVars(mInternalWorld);
}

void cSimulator::RemoveObj(int id)
{
	std::cout << "begin remove obj " << id << std::endl;
	mInternalWorld->removeCollisionObject(mSimObjs[id]);
	mSimObjs.erase(mSimObjs.begin() + id);
}

// extern const std::string rigidbody_path = "rigidbody.txt";
// extern const std::string multibody_path = "multiboody.txt";
// tVector before_angmom = tVector::Zero();
void cSimulator::StepSimulation(float dt)
{
	if (enable_bullet_sim)
	{
		mInternalWorld->stepSimulation(dt);
	}
	else
	{
		dt = 3e-3;
		if (mTime < 1e-10) PostUpdate(dt);

		// 1. use active force to update the velocity of the world
		ApplyActiveForce(dt);

		// 2. collision detect
		CollisionDetect();

		// 3. collision respose
		CollisionRespose(dt);

		// 4. update tranform
		UpdateTransform(dt);

		// std::cout << "step qdot = " << mMultibody->Getqdot().transpose() << std::endl;
		// 5. check momentums
		// tVector lin, ang;
		// for (auto& body : mSimObjs)
		// {
		// 	if (body->IsStatic() == true) continue;
		// 	std::ofstream fout(path, std::ios::app);

		// 	body->UpdateMomentum(lin, ang);

		// 	// // std::cout << "lin momentum = " << lin.transpose() << std::endl;
		// 	std::cout << "ang momentum = " << ang.transpose() << std::endl;
		// 	tVector diff = (before_angmom - ang);
		// 	before_angmom = ang;
		// 	fout << diff.transpose() << " " << diff.norm() / std::max(1e-3, before_angmom.norm())  <<  std::endl;
		// 	if (mTime > 1) exit(0);
		// }

		// 6. post update
		PostUpdate(dt);
	}
}
void cSimulator::PostUpdate(float dt)
{
	mTime += dt;
}
void cSimulator::GetContactInfo()
{
}

btDiscreteDynamicsWorld* cSimulator::GetInternalWorld()
{
	return mInternalWorld;
}

btBroadphaseInterface* cSimulator::GetBroadphase()
{
	return m_broadphase;
}
btCollisionDispatcher* cSimulator::GetDispatcher()
{
	return m_dispatcher;
}
btDefaultCollisionConfiguration* cSimulator::GetConfiguration()
{
	return m_collisionConfiguration;
}

void cSimulator::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const std::string& name, const btVector4& color)
{
	btVector3 localInertia(0, 0, 0);
	if (mass != 0.f)
		shape->calculateLocalInertia(mass, localInertia);

	cSimRigidBody::tParams params;
	params.col_shape = shape;
	params.damping = 0;
	params.mass = mass;
	params.name = name;
	params.local_inertia = cBulletUtil::btVectorTotVector0(localInertia);
	params.origin = cBulletUtil::btVectorTotVector1(startTransform.getOrigin());
	params.rot = cBulletUtil::btQuaternionTotQuaternion(startTransform.getRotation());

	btCollisionObject* obj = new cSimRigidBody(params);
	obj->setWorldTransform(startTransform);
	mSimObjs.push_back(dynamic_cast<cSimRigidBody*>(obj));
	// {
	// 	mSimObjs[0]->SetLinVel(tVector(0, -10, 0, 0));
	// }

	obj->setCollisionShape(shape);
	mInternalWorld->addCollisionObject(obj, 1, -1);
}

// 1. Apply active force, update the velocity, write into bullet
void cSimulator::ApplyActiveForce(float dt)
{
	// add gravity to simobjs and multibody
	for (auto& obj : mSimObjs) obj->ApplyCOMForce(obj->GetMass() * mGravity);

	if (mMultibody) mMultibody->ApplyGravity(mGravity);

	// add custom force
	// tVector force = tVector(1, 1, 1, 0);
	// tVector pos = tVector(1, 0, 0, 1);

	// if (mSimObjs.size() > 0)
	// 	mSimObjs[0]->ApplyForceOnGlobalPt(force, pos);
	// if (mMultibody)
	// 	mMultibody->ApplyForce(0, force, pos);
	// mMultibody->ApplyTorque(0, tVector(0, 1, 0, 0));

	// update vel
	UpdateVelocity(dt);
	ClearActiveForce();
}

// 2. collision detect, collect contact info
void cSimulator::CollisionDetect()
{
	mManifolds.clear();
	mInternalWorld->performDiscreteCollisionDetection();

	int num_of_manifolds = m_dispatcher->getNumManifolds();
	for (int i = 0; i < num_of_manifolds; i++) mManifolds.push_back(m_dispatcher->getManifoldByIndexInternal(i));
#ifdef __DEBUG__
	std::cout << "manifolds = " << num_of_manifolds << std::endl;
#endif
}

// 3. collision respose, calculate the contact force, update the velocity again
void cSimulator::CollisionRespose(float dt)
{
	// 4. collision response
	switch (mContactMode)
	{
		case eContactResponseMode::PenaltyMode:
			CollisionResposePenalty(dt);
			break;
		case eContactResponseMode::LCPMode:
			CollisionResposeLCP(dt);
		default:
			break;
	}

	// 5. update the vel
	// UpdateVelocity(dt);

	// 6. clear constraint force
	ClearActiveForce();
}

extern cSimRigidBody* UpcastRigidBody(const btCollisionObject* col);
extern cRobotCollider* UpcastRobotCollider(const btCollisionObject* col);

void cSimulator::CollisionResposePenalty(float dt)
{
	const float k = 2000;
	const float penetraction_threshold = 1e-3;
	for (auto& m : mManifolds)
	{
		// 1. get the collision pair
		int num_of_contacts = m->getNumContacts();
		cSimRigidBody *rigidbody0 = UpcastRigidBody(m->getBody0()),
					  *rigidbody1 = UpcastRigidBody(m->getBody1());
		cRobotCollider *mbody0 = nullptr, *mbody1 = nullptr;

		if (rigidbody0 == nullptr) mbody0 = UpcastRobotCollider(m->getBody0()), assert(mbody0 != nullptr);
		if (rigidbody1 == nullptr) mbody1 = UpcastRobotCollider(m->getBody1()), assert(mbody1 != nullptr);

		for (int i = 0; i < num_of_contacts; i++)
		{
			// 2. determine the contact normal on body0
			auto& pt = m->getContactPoint(i);

			tVector normal_on_0 = cBulletUtil::btVectorTotVector0(pt.m_normalWorldOnB);
			// std::cout << "body0 name = " << body0->GetName() << std::endl;

			// 3. penetration
			float penetraction = penetraction_threshold - pt.getDistance();
			// std::cout << "pene = " << penetraction << std::endl;

			// 4. apply the contact force
			tVector force_on_0 = penetraction * k * normal_on_0;
			// std::cout << "force on 0 = " << force_on_0.transpose() << std::endl;

			if (rigidbody0)
				rigidbody0->ApplyForce(force_on_0, cBulletUtil::btVectorTotVector1(pt.m_positionWorldOnA));
			else
				mbody0->mModel->ApplyForce(mbody0->mLinkId, force_on_0, cBulletUtil::btVectorTotVector0(pt.m_positionWorldOnA));

			if (rigidbody1)
				rigidbody1->ApplyForce(-force_on_0, cBulletUtil::btVectorTotVector1(pt.m_positionWorldOnB));
			else
				mbody1->mModel->ApplyForce(mbody1->mLinkId, -force_on_0, cBulletUtil::btVectorTotVector0(pt.m_positionWorldOnB));

			// body1->ApplyForce(-force_on_0, cBulletUtil::btVectorTotVector1(pt.m_positionWorldOnB) - body1->GetWorldPos());
			// exit(0);
		}
	}

	UpdateVelocity(dt);
}

void cSimulator::CollisionResposeLCP(float dt)
{
	mLCPContactSolver->ConstraintProcess(dt);
}

// 4. update transform, semi-implicit
void cSimulator::UpdateTransform(float dt)
{
	for (auto& obj : mSimObjs)
	{
		obj->UpdateTransform(dt);
		obj->WriteTransAndVelToBullet();
	}

	if (mMultibody) mMultibody->UpdateTransform(dt);
}

void cSimulator::UpdateVelocity(float dt)
{
	for (auto& obj : mSimObjs)
		obj->UpdateVelocity(dt);
	if (mMultibody)
	{
		// std::cout << "gravity update vel begin\n";
		mMultibody->UpdateVelocity(dt);
		// std::cout << "gravity update vel end\n";
	}
}

void cSimulator::ClearActiveForce()
{
	for (auto& obj : mSimObjs)
	{
		obj->ClearForce();
	}

	if (mMultibody)
	{
		mMultibody->ClearForces();
	}
}

btBoxShape* cSimulator::createBoxShape(const btVector3& halfExtents)
{
	btBoxShape* box = new btBoxShape(halfExtents);
	return box;
}