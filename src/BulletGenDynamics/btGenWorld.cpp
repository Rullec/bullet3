#include "btGenWorld.h"
#include "btGenModel/SimObj.h"
#include "btGenSolver/ContactSolver.h"
#include "btGenModel/RobotModelDynamics.h"
#include "btGenUtil/MathUtil.h"
#include "btGenUtil/JsonUtil.h"
#include <iostream>
#include <fstream>
// #define __DEBUG__

extern int global_frame_id;
std::map<int, std::string> col_name;
// const std::string path = "momentum.txt";
bool enable_bullet_sim = false;
// const std::string output_path = "multibody_0.rec";
extern bool gPauseSimulation;
// const std::string log_path = "debug_solve_new.txt";
btGeneralizeWorld::btGeneralizeWorld(const std::string& config_path)
{
	Json::Value config_js;
	cJsonUtil::LoadJson(config_path, config_js);
	mInternalWorld = nullptr;
	m_broadphase = nullptr;
	m_dispatcher = nullptr;
	m_collisionConfiguration = nullptr;
	mSimObjs.clear();
	mMultibody = nullptr;
	mTime = 0;
	{
		bool enable_gravity = cJsonUtil::ParseAsBool("enable_gravity", config_js);
		if (enable_gravity)
		{
			mGravity = tVector(0, -9.8, 0, 0);
		}
		else
		{
			mGravity.setZero();
		}

		bool enable_lcp = cJsonUtil::ParseAsBool("enable_lcp", config_js);
		if (enable_lcp == true)
			mContactMode = eContactResponseMode::LCPMode;
		else
			mContactMode = eContactResponseMode::PenaltyMode;
		mRigidDamping = cJsonUtil::ParseAsDouble("rigid_damping", config_js);

		mMBDamping1 = cJsonUtil::ParseAsDouble("mb_damping1", config_js);
		mMBDamping2 = cJsonUtil::ParseAsDouble("mb_damping2", config_js);
		mMBZeroInitPose = cJsonUtil::ParseAsBool("mb_zero_init_pose", config_js);
		mMBZeroInitPoseVel = cJsonUtil::ParseAsBool("mb_zero_init_pose_vel", config_js);
		mMBTestJacobian = cJsonUtil::ParseAsBool("mb_test_jacobian", config_js);
		mMBEAngleClamp = cJsonUtil::ParseAsBool("mb_angle_clamp", config_js);
		mMBEnableCollectFrameInfo = cJsonUtil::ParseAsBool("mb_enable_collect_frame_info", config_js);
		mMBCollectFrameNum = cJsonUtil::ParseAsInt("mb_collect_frame_num", config_js);
		mEnablePauseWhenMaxVel = cJsonUtil::ParseAsBool("enable_pause_when_max_vel", config_js);

		mMBScale = cJsonUtil::ParseAsDouble("mb_scale", config_js);
		mMBEnableRk4 = cJsonUtil::ParseAsBool("enable_RK4_for_multibody", config_js);
		mMBEpsDiagnoalMassMat = cJsonUtil::ParseAsDouble("mb_add_eps_at_diagnoal_mass_mat", config_js);
		mMBMaxVel = cJsonUtil::ParseAsDouble("mb_max_vel", config_js);
		mMBUpdateVelWithoutCoriolis = cJsonUtil::ParseAsDouble("mb_update_velocity_without_coriolis", config_js);
		mEnablePeturb = cJsonUtil::ParseAsDouble("enable_perturb", config_js);
		mLCPConfigPath = cJsonUtil::ParseAsString("lcp_config_path", config_js);
	}
	mLCPContactSolver = nullptr;
	mManifolds.clear();

	mFrameInfo.clear();

	// std::ofstream fout(log_path);
	// fout << "";
	// fout.close();
}
btGeneralizeWorld::~btGeneralizeWorld()
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

void btGeneralizeWorld::Init()
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
		mLCPContactSolver = new cContactSolver(mLCPConfigPath, mInternalWorld);
	}
}

void btGeneralizeWorld::AddGround()
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

void btGeneralizeWorld::AddObj(int n, const std::string& obj_type, bool add_perturb /*=false*/)
{
	btCollisionShape* colShape = nullptr;
	if (obj_type == "ball")
	{
		colShape = new btSphereShape(btScalar(0.5));
		btCompoundShape* compunde = new btCompoundShape();
		compunde->addChildShape(btTransform::getIdentity(), colShape);
	}
	else if (obj_type == "cube")
	{
		colShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
	}
	else if (obj_type == "stick")
	{
		colShape = new btBoxShape(btVector3(0.001, 0.1, 2));
	}
	else
	{
		std::cout << "unsupported type " << obj_type << std::endl;
		exit(1);
	}

	/// Create Dynamic Objects
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0, -1, 0));
	// startTransform.setOrigin(btVector3(0.680375, -1.18218, 0.566198));
	// tQuaternion qua = tQuaternion(0.800701, 0.372043, 0.28516, -0.373023);
	// startTransform.setRotation(cBulletUtil::tQuaternionTobtQuaternion(qua));

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
					btScalar(-1 + 1.1 * k),
					btScalar(1 * j)));
				if (add_perturb) startTransform.setOrigin(startTransform.getOrigin() + cBulletUtil::tVectorTobtVector(tVector::Random() * 0.1));
				createRigidBody(1.f, startTransform, colShape, obj_type + std::to_string(k));
			}
		}
	}
	std::cout << "obj num = " << mSimObjs.size() << std::endl;
	// mSimObjs[0]-(tVector(0, -0.607199, 0, 0));
	// mSimObjs[0]->SetAngVel(tVector(2.14574, 0.00479028, -0.277455, 0));
	// mSimObjs[0]->set(tVector(0, -0.607168, 0, 0));
}

// void btGeneralizeWorld::AddMultibody(int num)
void btGeneralizeWorld::AddMultibody(const std::string& skeleton_name)
{
	if (mMultibody != nullptr) delete mMultibody;

	// const std::string prefix = "../DeepMimic/data/0424/characters/";
	// std::string skeleton_name = prefix;
	// switch (num)
	// {
	// 	case 0:
	// 		skeleton_name += "skeleton_box.json";
	// 		break;
	// 	case 1:
	// 		skeleton_name += "skeleton_042302_revised_0_obj.json";
	// 		break;
	// 	case 2:
	// 		skeleton_name += "skeleton_042302_revised_1_obj.json";
	// 		break;
	// 	case 3:
	// 		skeleton_name += "skeleton_042302_revised_2_obj.json";
	// 		break;
	// 	case 4:
	// 		skeleton_name += "skeleton_042302_revised_leftleg.json";
	// 		break;
	// 	case 5:
	// 		skeleton_name += "skeleton_042302_4_links.json";
	// 		break;
	// 	case 8:
	// 		skeleton_name += "skeleton_042302_revised_legs.json";
	// 		break;
	// 	case 20:
	// 		skeleton_name += "skeleton_042302_revised.json";
	// 		break;
	// 	default:
	// 		skeleton_name += "skeleton_sphere.json";
	// 		break;
	// }
	mMultibody = new cRobotModelDynamics(skeleton_name.c_str(), mMBScale, ModelType::JSON);
	mMultibody->SetComputeSecondDerive(true);
	mMultibody->SetDampingCoeff(mMBDamping1, mMBDamping2);
	mMultibody->SetAngleClamp(mMBEAngleClamp);
	mMultibody->SetMaxVel(mMBMaxVel);

	mMultibody->InitSimVars(mInternalWorld, mMBZeroInitPose, mMBZeroInitPoseVel);
	if (mMBTestJacobian)
	{
		mMultibody->TestJacobian();
		mMultibody->TestSecondJacobian();
		exit(0);
	}
}

#include "BulletGenDynamics/btGenUtil/TimeUtil.hpp"
void btGeneralizeWorld::RemoveObj(int id)
{
	std::cout << "begin remove obj " << id << std::endl;
	mInternalWorld->removeCollisionObject(mSimObjs[id]);
	mSimObjs.erase(mSimObjs.begin() + id);
}

// extern const std::string rigidbody_path = "rigidbody.txt";
// extern const std::string multibody_path = "multiboody.txt";
// tVector before_angmom = tVector::Zero();
void OutputJacobianTestJson(cRobotModelDynamics*, const std::string& path);
void OutputDynamicsFTestJson(cRobotModelDynamics*, const std::string& path);
void btGeneralizeWorld::StepSimulation(double dt)
{
	cTimeUtil::Begin("stepsim");
	std::cout << "------------------begin step simulation for frame " << global_frame_id << " time " << mTime << "------------------\n";

	if (enable_bullet_sim)
	{
		mInternalWorld->stepSimulation(dt);
	}
	else
	{
		ApplyActiveForce(dt);

		CollisionDetect();

		cTimeUtil::Begin("collision_response");
		CollisionRespose(dt);
		cTimeUtil::End("collision_response");

		Update(dt);

		PostUpdate(dt);
	}
	cTimeUtil::End("stepsim");
}
void btGeneralizeWorld::PostUpdate(double dt)
{
	mTime += dt;
}
void btGeneralizeWorld::GetContactInfo()
{
}

btDiscreteDynamicsWorld* btGeneralizeWorld::GetInternalWorld()
{
	return mInternalWorld;
}

btBroadphaseInterface* btGeneralizeWorld::GetBroadphase()
{
	return m_broadphase;
}
btCollisionDispatcher* btGeneralizeWorld::GetDispatcher()
{
	return m_dispatcher;
}
btDefaultCollisionConfiguration* btGeneralizeWorld::GetConfiguration()
{
	return m_collisionConfiguration;
}

void btGeneralizeWorld::createRigidBody(double mass, const btTransform& startTransform, btCollisionShape* shape, const std::string& name, const btVector4& color)
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

#include "BulletGenDynamics/btGenModel/EulerAngelRotationMatrix.h"
// 1. Apply active force, update the velocity, write into bullet
void btGeneralizeWorld::ApplyActiveForce(double dt)
{
	// BT_PROFILE("apply_active_force");
	// add gravity to simobjs and multibody
	for (auto& obj : mSimObjs)
	{
		obj->ApplyCOMForce(obj->GetMass() * mGravity);
		// std::cout << "obj apply " << (obj->GetMass() * mGravity).transpose() << std::endl;
	}

	if (mMultibody)
	{
		mMultibody->ApplyGravity(mGravity);

		// std::cout << "q = " << mMultibody->Getq().transpose() << std::endl;
		// std::cout << "qdot = " << mMultibody->Getqdot().transpose() << std::endl;
	}

	if (mEnablePeturb)
	{
		if (mMultibody)
		{
			for (int i = 0; i < mMultibody->GetNumOfLinks(); i++)
			{
				mMultibody->ApplyJointTorque(i, tVector::Random() / 10);
				mMultibody->ApplyForce(std::rand() % mMultibody->GetNumOfLinks(), tVector::Random(), tVector::Random());
				mMultibody->ApplyLinkTorque(i, tVector::Random());
			}
		}
		std::cout << "perturb Q = " << mMultibody->GetGeneralizedForce().transpose() << std::endl;
	}
}

// 2. collision detect, collect contact info
void btGeneralizeWorld::CollisionDetect()
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
void btGeneralizeWorld::CollisionRespose(double dt)
{
	// no collision, return
	// if (mManifolds.size() == 0) return;
	// int num_contact_points = 0;
	// for(auto & x : mManifolds) num_contact_points += x->getNumContacts();
	// if (num_contact_points == 0) return;

	// here, we must record the pre-collsiion state if the convert_mat_test is enabled in the LCPSolver.

	// PushStatePreCollision();

	// update velocity by active force now
	if (false == mMBUpdateVelWithoutCoriolis)
	{
		UpdateVelocityInternal(dt);
	}
	else
	{
		UpdateVelocityInternalWithoutCoriolis(dt);
	}

	// because "UpdateVelocity" is needed in the LCP test function as well, so we need to clear active force here as well
	ClearForce();

	// std::cout << "[before lcp] q = " << mMultibody->Getq().transpose() << std::endl;

	// std::cout << "[before lcp] qdot = " << mMultibody->Getqdot().transpose() << std::endl;
	// exit(1);
	// tVectorXd qold = mMultibody->Getq();
	// tVectorXd qdotold = mMultibody->Getqdot();
	// tVectorXd qddotold = mMultibody->Getqddot();
	// 4. collision response
	cTimeUtil::Begin("LCP total");
	switch (mContactMode)
	{
		case eContactResponseMode::PenaltyMode:
			CollisionResposePenalty(dt);
			break;
		case eContactResponseMode::LCPMode:
			CollisionResposeLCP(dt);
		case eContactResponseMode::SequentialImpulseMode:

		default:
			break;
	}
	cTimeUtil::End("LCP total");
	// tVectorXd qnew = mMultibody->Getq();
	// tVectorXd qdotnew = mMultibody->Getqdot();
	// tVectorXd qddotnew = mMultibody->Getqddot();
	// double qdiff = (qnew - qold).norm(),
	// 	   qdotdiff = (qdotnew - qdotold).norm(),
	// 	   qddotdiff = (qddotnew - qddotold).norm();
	// std::cout << "q diff = " << qdiff << std::endl;
	// std::cout << "qdot diff = " << qdotdiff << std::endl;
	// std::cout << "qddot diff = " << qddotdiff << std::endl;

	// restore state
	// PopStatePostColliison();

	// apply these contact forces
	// std::cout << "[before add contact force] Q = " << mMultibody->GetGeneralizedForce().transpose() << std::endl;
	for (auto& f : mContactForces)
	{
		// std::cout << "add contact f = " << f->mForce.transpose() << std::endl;
		f->mObj->ApplyForce(f->mForce, f->mWorldPos);
	}

	// apply contact torques
	for (auto& t : mConstraintGenalizedForce)
	{
		// std::cout << "[sim] model dof " << t->dof_id << " add constraint generalized force " << t->value << std::endl;
		// t->model->ApplyJointTorque(t->joint_id, t->joint_torque);
		t->model->ApplyGeneralizedForce(t->dof_id, t->value);
	}
	// std::cout << "[after add contact force] Q = " << mMultibody->GetGeneralizedForce().transpose() << std::endl;
	// std::ofstream fout(log_path, std::ios::app);
	// fout << "------frame " << global_frame_id << "------\n";
	// fout << "q = " << mMultibody->Getq().transpose() << std::endl;
	// fout << "qdot = " << mMultibody->Getqdot().transpose() << std::endl;
	// fout << "Q = " << mMultibody->GetGeneralizedForce().transpose() << std::endl;
	// fout.close();
}

extern cSimRigidBody* UpcastRigidBody(const btCollisionObject* col);
extern cRobotCollider* UpcastRobotCollider(const btCollisionObject* col);

void btGeneralizeWorld::CollisionResposePenalty(double dt)
{
	const double k = 2000;
	const double penetraction_threshold = 1e-3;
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
			double penetraction = penetraction_threshold - pt.getDistance();
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

	UpdateVelocityInternal(dt);
}

void btGeneralizeWorld::CollisionResposeLCP(double dt)
{
	mLCPContactSolver->ConstraintProcess(dt);
	mContactForces = mLCPContactSolver->GetContactForces();
	mConstraintGenalizedForce = mLCPContactSolver->GetConstraintGeneralizedForces();
	// if (mContactTorques.size()) std::cout << "[sim] get constraint torque 0 = " << mContactTorques[0]->joint_torque.transpose() << std::endl;
	// exit(0);
}

void btGeneralizeWorld::CollisionResposeSI(double dt)
{
}

// 4. update transform, semi-implicit
// void btGeneralizeWorld::UpdateTransform(double dt)
// {
// 	for (auto& obj : mSimObjs)
// 	{
// 		obj->UpdateTransform(dt);
// 		obj->WriteTransAndVelToBullet();
// 	}

// 	if (mMultibody) mMultibody->UpdateTransform(dt);
// }

void btGeneralizeWorld::UpdateVelocityInternal(double dt)
{
	for (auto& obj : mSimObjs)
		obj->UpdateVelocity(dt);
	if (mMultibody)
	{
		mMultibody->UpdateVelocity(dt);
	}
}

void btGeneralizeWorld::UpdateVelocityInternalWithoutCoriolis(double dt)
{
	for (auto& obj : mSimObjs)
		obj->UpdateVelocity(dt);
	if (mMultibody)
	{
		mMultibody->UpdateVelocityWithoutCoriolis(dt);
	}
}

void btGeneralizeWorld::Update(double dt)
{
	// all forces has been recorded now
	if (mMBEnableCollectFrameInfo)
	{
		if (global_frame_id < mMBCollectFrameNum)
			CollectFrameInfo(dt);
		if (global_frame_id == mMBCollectFrameNum)
		{
			gPauseSimulation = true;
			WriteFrameInfo("frame_info.txt");
		}
	}

	// for rigidbodies
	for (auto& obj : mSimObjs)
	{
		obj->UpdateVelocity(dt);
		obj->UpdateTransform(dt);
		obj->WriteTransAndVelToBullet();
	}

	// for multibody
	if (mMultibody)
	{
		if (mMBEnableRk4)
		{
			mMultibody->UpdateRK4(dt);
		}
		else
		{
			mMultibody->UpdateVelocity(dt, true);
			mMultibody->UpdateTransform(dt);
		}
		if (mMultibody->IsMaxVel())
		{
			std::cout << "[warn] multibody exceed max vel = " << mMultibody->Getqdot().cwiseAbs().maxCoeff()
					  << std::endl;
			if (mEnablePauseWhenMaxVel) gPauseSimulation = true;
			// exit(1);
		}
	}
	ClearForce();
}

void btGeneralizeWorld::ClearForce()
{
	for (auto& obj : mSimObjs)
	{
		obj->ClearForce();
	}

	if (mMultibody)
	{
		mMultibody->ClearForce();
	}
}

btBoxShape* btGeneralizeWorld::createBoxShape(const btVector3& halfExtents)
{
	btBoxShape* box = new btBoxShape(halfExtents);
	return box;
}

void btGeneralizeWorld::PushStatePreCollision()
{
	// std::cout << "here, only push & pop velocity and force state\n";
	std::string name = "pre_collision";
	for (auto& obj : mSimObjs)
	{
		obj->PushState(name, false);
	}

	if (mMultibody)
	{
		mMultibody->PushState(name, false);
	}
}

void btGeneralizeWorld::PopStatePostColliison()
{
	// std::cout << "here, only push & pop velocity and force state\n";
	std::string name = "pre_collision";
	for (auto& obj : mSimObjs)
	{
		obj->PopState(name, false);
	}

	if (mMultibody)
	{
		mMultibody->PopState(name, false);
	}
}

void btGeneralizeWorld::CollectFrameInfo(double dt)
{
	tFrameInfo info;
	info.force_array.clear();
	info.torque_array.clear();
	info.frame_id = global_frame_id;
	info.timestep = dt;
	mMultibody->GetEffectInfo(info.force_array, info.torque_array);
	info.q = mMultibody->Getq();
	info.qdot = mMultibody->Getqdot();

	info.Q = mMultibody->GetGeneralizedForce();
	info.mass_mat = mMultibody->GetMassMatrix();
	info.mass_mat_inv = mMultibody->GetInvMassMatrix();
	info.coriolis_mat = mMultibody->GetCoriolisMatrix();
	info.damping_mat = mMultibody->GetDampingMatrix();

	info.residual = info.Q - (info.coriolis_mat + info.damping_mat) * info.qdot;
	info.qddot = mMultibody->Getqddot();
	mFrameInfo.push_back(info);
}

void btGeneralizeWorld::WriteFrameInfo(const std::string& path)
{
	Json::Value root;
	std::ofstream fout(path);
	root["link_num"] = mMultibody->GetNumOfLinks();
	root["dof_num"] = mMultibody->GetNumOfFreedom();
	root["frame_num"] = mFrameInfo.size();
	Json::Value FrameArray = Json::arrayValue;
	for (auto& item : mFrameInfo)
	{
		Json::Value FrameValue;
		FrameValue["frame_id"] = item.frame_id;
		FrameValue["q"] = cJsonUtil::BuildVectorJsonValue(item.q);
		FrameValue["qdot"] = cJsonUtil::BuildVectorJsonValue(item.qdot);
		FrameValue["qddot"] = cJsonUtil::BuildVectorJsonValue(item.qddot);
		FrameValue["Q"] = cJsonUtil::BuildVectorJsonValue(item.Q);
		FrameValue["residual"] = cJsonUtil::BuildVectorJsonValue(item.residual);
		FrameValue["timestep"] = item.timestep;

		for (int i = 0; i < mMultibody->GetNumOfLinks(); i++)
		{
			FrameValue["force_array"].append(cJsonUtil::BuildVectorJsonValue(item.force_array[i]));
			FrameValue["torque_array"].append(cJsonUtil::BuildVectorJsonValue(item.torque_array[i]));
		}
		FrameValue["mass_mat"] = cJsonUtil::BuildMatrixJsonValue(item.mass_mat);
		FrameValue["mass_mat_inv"] = cJsonUtil::BuildMatrixJsonValue(item.mass_mat_inv);
		FrameValue["coriolis_mat"] = cJsonUtil::BuildMatrixJsonValue(item.coriolis_mat);
		FrameValue["damping_mat"] = cJsonUtil::BuildMatrixJsonValue(item.damping_mat);
		FrameArray.append(FrameValue);
	}
	root["frame_array"] = FrameArray;
	cJsonUtil::WriteJson(path, root, true);
	fout.close();
}

void OutputJacobianTestJson(cRobotModelDynamics* mb, const std::string& path)
{
	Json::Value root;

	root["q"] = Json::arrayValue;
	root["qdot"] = Json::arrayValue;
	for (int i = 0; i < mb->GetNumOfFreedom(); i++)
	{
		root["q"].append(mb->Getq()[i]);
		root["qdot"].append(mb->Getqdot()[i]);
	}

	int num_of_links = mb->GetNumOfLinks();
	root["link_num"] = num_of_links;
	Json::Value link_json;
	tMatrixXd jac_buf;
	for (int i = 0; i < num_of_links; i++)
	{
		auto link = mb->GetLinkById(i);
		tVector local_pos = tVector::Random();
		// tVector local_pos = tVector::Zero();
		local_pos[3] = 1;
		tVector global_pos = link->GetGlobalTransform() * local_pos;

		link_json["local_pos"] = cJsonUtil::BuildVectorJsonValue(local_pos);
		link_json["global_pos"] = cJsonUtil::BuildVectorJsonValue(global_pos);
		mb->ComputeJacobiByGivenPointTotalDOFWorldFrame(i, global_pos.segment(0, 3), jac_buf);
		link_json["jacobian"] = cJsonUtil::BuildMatrixJsonValue(jac_buf);
		link_json["world_trans"] = cJsonUtil::BuildMatrixJsonValue(link->GetGlobalTransform());
		root["link_" + std::to_string(i)] = link_json;
	}

	cJsonUtil::WriteJson(path.c_str(), root, true);
}

void OutputDynamicsFTestJson(cRobotModelDynamics* mb, const std::string& path)
{
	Json::Value root;

	root["q"] = Json::arrayValue;
	root["qdot"] = Json::arrayValue;
	std::cout << "[log] q = " << mb->Getq().transpose() << std::endl;
	std::cout << "[log] qdot = " << mb->Getqdot().transpose() << std::endl;
	for (int i = 0; i < mb->GetNumOfFreedom(); i++)
	{
		root["q"].append(mb->Getq()[i]);
		root["qdot"].append(mb->Getqdot()[i]);
	}

	int num_of_links = mb->GetNumOfLinks();
	root["link_num"] = num_of_links;
	Json::Value link_json;
	mb->ClearForce();
	for (int i = 0; i < num_of_links; i++)
	{
		auto link = mb->GetLinkById(i);
		tVector local_pos = tVector::Random();
		local_pos[3] = 1;
		tVector global_pos = link->GetGlobalTransform() * local_pos;
		tVector global_force = tVector::Random();

		link_json["force_local_pos"] = cJsonUtil::BuildVectorJsonValue(local_pos);
		link_json["force_global_pos"] = cJsonUtil::BuildVectorJsonValue(global_pos);
		link_json["force_global"] = cJsonUtil::BuildVectorJsonValue(global_force.segment(0, 3));
		mb->ApplyForce(i, global_force, global_pos);

		// link_json["jacobian"] = cJsonUtil::BuildMatrixJsonValue(jac_buf);
		root["link_" + std::to_string(i)] = link_json;
	}
	root["generalized_force"] = cJsonUtil::BuildVectorJsonValue(mb->GetGeneralizedForce());

	root["mass_mat"] = cJsonUtil::BuildMatrixJsonValue(mb->GetMassMatrix());
	root["coriolis_mat"] = cJsonUtil::BuildMatrixJsonValue(mb->GetCoriolisMatrix());

	tVectorXd qddot = mb->GetInvMassMatrix() * (mb->GetGeneralizedForce() - mb->GetCoriolisMatrix() * mb->Getqdot());
	root["qddot"] = cJsonUtil::BuildVectorJsonValue(qddot);
	cJsonUtil::WriteJson(path.c_str(), root, true);
}