#include "btGenWorld.h"
#include "BulletGenDynamics/btGenController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/btTraj.h"
#include "BulletGenDynamics/btGenModel/EulerAngelRotationMatrix.h"
#include "BulletGenDynamics/btGenUtil/TimeUtil.hpp"
#include "btGenController/btGenContactAwareAdviser.h"
#include "btGenModel/RobotModelDynamics.h"
#include "btGenModel/SimObj.h"
#include "btGenSolver/ContactSolver.h"
#include "btGenUtil/JsonUtil.h"
#include "btGenUtil/MathUtil.h"
#include <fstream>
#include <iostream>
// #define __DEBUG__

int global_frame_id = 0;
std::map<int, std::string> col_name;
// const std::string path = "momentum.txt";
bool enable_bullet_sim = false;
// const std::string output_path = "multibody_0.rec";
// extern bool gPauseSimulation;
// const std::string log_path = "debug_solve_new.txt";
btGeneralizeWorld::btGeneralizeWorld()
{
    // mPDController = nullpt
    mCollisionShapeArray.clear();
    mInternalWorld = nullptr;
    m_broadphase = nullptr;
    m_dispatcher = nullptr;
    m_collisionConfiguration = nullptr;
    mSimObjs.clear();
    mMultibody = nullptr;

    mGuideTraj = nullptr;
    mFrameId = 0;
}

btGeneralizeWorld::~btGeneralizeWorld()
{
    // std::cout << "Begin simulator deconstrutor\n";
    if (mInternalWorld)
    {
        // 1. remove rigid objs
        for (int i = 0; i < mSimObjs.size(); i++)
        {
            mInternalWorld->removeCollisionObject(mSimObjs[i]);
            delete mSimObjs[i];
        }
        mSimObjs.clear();

        // 2. remove collider
        if (mMultibody)
        {
            mMultibody->GetLinkCollider(0);
            delete mMultibody;
        }

        // 3. remove the world
        delete mInternalWorld;
    }

    delete m_broadphase;
    delete m_dispatcher;
    delete m_collisionConfiguration;
    mManifolds.clear();
    for (auto &x : mCollisionShapeArray)
        delete x;
    mCollisionShapeArray.clear();

    delete mLCPContactSolver;
    delete mGuideTraj;
    delete mControlAdviser;
}

void btGeneralizeWorld::Init(const std::string &config_path)
{
    {
        Json::Value config_js;
        btJsonUtil::LoadJson(config_path, config_js);
        mInternalWorld = nullptr;
        m_broadphase = nullptr;
        m_dispatcher = nullptr;
        m_collisionConfiguration = nullptr;
        mSimObjs.clear();
        mMultibody = nullptr;
        mTime = 0;
        {
            bool enable_gravity =
                btJsonUtil::ParseAsBool("enable_gravity", config_js);
            if (enable_gravity)
            {
                mGravity = tVector(0, -9.8, 0, 0);
            }
            else
            {
                mGravity.setZero();
            }

            std::string contact_response_mode =
                btJsonUtil::ParseAsString("contact_response_mode", config_js);
            if (contact_response_mode == "LCP")
            {
                mContactMode = eContactResponseMode::LCPMode;
            }
            else if (contact_response_mode == "Penalty")
            {
                mContactMode = eContactResponseMode::PenaltyMode;
            }
            else if (contact_response_mode == "No")
            {
                mContactMode = eContactResponseMode::NoMode;
            }
            else
            {
                std::cout << "[error] Unsupported contact response mode "
                          << contact_response_mode << std::endl;
                exit(1);
            }

            mRigidDamping =
                btJsonUtil::ParseAsDouble("rigid_damping", config_js);

            mMBDamping1 = btJsonUtil::ParseAsDouble("mb_damping1", config_js);
            mMBDamping2 = btJsonUtil::ParseAsDouble("mb_damping2", config_js);
            mMBZeroInitPose =
                btJsonUtil::ParseAsBool("mb_zero_init_pose", config_js);
            mMBZeroInitPoseVel =
                btJsonUtil::ParseAsBool("mb_zero_init_pose_vel", config_js);
            // mMBTestJacobian = btJsonUtil::ParseAsBool("mb_test_jacobian",
            // config_js); mMBEAngleClamp =
            // btJsonUtil::ParseAsBool("mb_angle_clamp", config_js);
            mMBEnableCollectFrameInfo = btJsonUtil::ParseAsBool(
                "mb_enable_collect_frame_info", config_js);
            mMBCollectFrameNum =
                btJsonUtil::ParseAsInt("mb_collect_frame_num", config_js);
            mEnablePauseWhenMaxVel =
                btJsonUtil::ParseAsBool("enable_pause_when_max_vel", config_js);

            mMBScale = btJsonUtil::ParseAsDouble("mb_scale", config_js);
            // mMBEnableRk4 =
            // btJsonUtil::ParseAsBool("enable_RK4_for_multibody", config_js);
            mMBEpsDiagnoalMassMat = btJsonUtil::ParseAsDouble(
                "mb_add_eps_at_diagnoal_mass_mat", config_js);
            mMBMaxVel = btJsonUtil::ParseAsDouble("mb_max_vel", config_js);
            mMBEnableContactAwareLCP = false;
            // mMBEnableContactAwareLCP = btJsonUtil::ParseAsBool(
            //     "mb_enable_contact_aware_lcp", config_js);
            // mMBEnableGuideAction =
            // btJsonUtil::ParseAsBool("mb_enable_guide_action", config_js);
            // mGuidedActionTrajFile =
            // btJsonUtil::ParseAsString("guided_action_traj_file", config_js);
            // if (mMBEnableGuideAction && mMBEnableContactAwareLCP)
            // {
            // 	std::cout << "[error] the guided action & contact aware LCP
            // cannot be opened simutaneously.\n"; 	exit(0);
            // }
            mContactAwareConfig = btJsonUtil::ParseAsString(
                "contact_aware_lcp_config", config_js);
            // mMBContactAwareW =
            // btJsonUtil::ParseAsDouble("mb_contact_aware_lcp_W", config_js);
            // mMBContactAwareWm =
            // btJsonUtil::ParseAsDouble("mb_contact_aware_lcp_Wm", config_js);
            // mContactAwareGuideTraj =
            // btJsonUtil::ParseAsString("contact_aware_guide_traj", config_js);

            // mMBUpdateVelWithoutCoriolis =
            // btJsonUtil::ParseAsDouble("mb_update_velocity_without_coriolis",
            // config_js);
            mEnablePeturb =
                btJsonUtil::ParseAsDouble("enable_perturb", config_js);
            // mEnablePDControl = btJsonUtil::ParseAsBool("enable_pd_control",
            // config_js); mPDControllerPath =
            // btJsonUtil::ParseAsString("controller_path", config_js);
            mLCPConfigPath =
                btJsonUtil::ParseAsString("lcp_config_path", config_js);
        }

        mLCPContactSolver = nullptr;
        mManifolds.clear();

        mFrameInfo.clear();
    }

    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    m_broadphase = new btDbvtBroadphase();
    btConstraintSolver *solver = nullptr;
    if (enable_bullet_sim == true)
    {
        solver = new btSequentialImpulseConstraintSolver();
    }

    mInternalWorld = new btDiscreteDynamicsWorld(
        m_dispatcher, m_broadphase, solver, m_collisionConfiguration);

    if (mContactMode == LCPMode)
    {
        mLCPContactSolver =
            new btGenContactSolver(mLCPConfigPath, mInternalWorld);
    }

    // if (mEnablePDControl)
    // {
    // 	mPDController = new btGenPDController(mPDControllerPath);
    // }
    mControlAdviser = nullptr;
    // mControlAdviser = new btGenContactAwareAdviser();
    // SetEnableContacrAwareControl();
}

void btGeneralizeWorld::AddGround(double height)
{
    {
        btStaticPlaneShape *plane =
            new btStaticPlaneShape(btVector3(0, 1, 0), height);
        mCollisionShapeArray.push_back(plane);
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(btVector3(0, 0, 0));
        createRigidBody(0, trans, plane, "ground");
    }
    // {
    // 	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),
    // btScalar(50.), btScalar(50.)));

    // 	/// Create Dynamic Objects
    // 	btTransform startTransform;
    // 	startTransform.setIdentity();
    // 	startTransform.setOrigin(btVector3(0, -52, 0));

    // 	// startTransform.setRotation(btQuaternion(btVector3(1, 0, 0), SIMD_PI
    // / 6.)); 	createRigidBody(0.f, startTransform, groundShape, "ground");
    // }
}

void btGeneralizeWorld::AddObj(int n, const std::string &obj_type,
                               bool add_perturb /*=false*/)
{
    btCollisionShape *colShape = nullptr;
    if (obj_type == "ball")
    {
        colShape = new btSphereShape(btScalar(0.5));
        btCompoundShape *compunde = new btCompoundShape();
        compunde->addChildShape(btTransform::getIdentity(), colShape);
        mCollisionShapeArray.push_back(compunde);
    }
    else if (obj_type == "cube")
    {
        colShape = new btBoxShape(btVector3(0.5, 0.5, 0.5));
        mCollisionShapeArray.push_back(colShape);
    }
    else if (obj_type == "stick")
    {
        colShape = new btBoxShape(btVector3(0.001, 0.1, 2));
        mCollisionShapeArray.push_back(colShape);
    }
    else
    {
        std::cout << "unsupported type " << obj_type << std::endl;
        exit(0);
    }

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();
    startTransform.setOrigin(btVector3(0, -1, 0));
    // startTransform.setOrigin(btVector3(0.680375, -1.18218, 0.566198));
    // tQuaternion qua = tQuaternion(0.800701, 0.372043, 0.28516, -0.373023);
    // startTransform.setRotation(btBulletUtil::tQuaternionTobtQuaternion(qua));

    // startTransform.setRotation(btQuaternion(btVector3(1, 1, 0), SIMD_PI
    // / 6.));

    // for (int i = 0; i < n; i++)
    // {
    // 	startTransform.setOrigin(startTransform.getOrigin() + btVector3(0, 1.5,
    // 0)); 	createRigidBody(1.f, startTransform, colShape, "ball" +
    // std::to_string(i));
    // }

    for (int k = 0; k < n; k++)
    {
        for (int i = 0; i < 1; i++)
        {
            for (int j = 0; j < 1; j++)
            {
                startTransform.setOrigin(btVector3(
                    btScalar(1 * i), btScalar(-1 + 1.1 * k), btScalar(1 * j)));
                if (add_perturb)
                    startTransform.setOrigin(startTransform.getOrigin() +
                                             btBulletUtil::tVectorTobtVector(
                                                 tVector::Random() * 0.1));
                createRigidBody(1.f, startTransform, colShape,
                                obj_type + std::to_string(k));
            }
        }
    }
    std::cout << "obj num = " << mSimObjs.size() << std::endl;
    // mSimObjs[0]-(tVector(0, -0.607199, 0, 0));
    // mSimObjs[0]->SetAngVel(tVector(2.14574, 0.00479028, -0.277455, 0));
    // mSimObjs[0]->set(tVector(0, -0.607168, 0, 0));
}

void btGeneralizeWorld::AddStaticBody(btCollisionObject *obj, double mass,
                                      const std::string &name)
{
    createRigidBody(mass, obj->getWorldTransform(), obj->getCollisionShape(),
                    name);
}
void btGeneralizeWorld::RemoveStaticBody(btCollisionObject *obj)
{
    std::cout << "remove static body hasn't been implemented\n";
}
void btGeneralizeWorld::SetGravity(const tVector &g) { mGravity = g; }

tVector btGeneralizeWorld::GetGravity() const { return mGravity; }
// void btGeneralizeWorld::AddMultibody(int num)
void btGeneralizeWorld::AddMultibody(const std::string &skeleton_name)
{
    if (mMultibody != nullptr)
        delete mMultibody;

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
    mMultibody = new cRobotModelDynamics();
    mMultibody->Init(skeleton_name.c_str(), mMBScale, ModelType::JSON);
    mMultibody->SetComputeSecondDerive(true);
    mMultibody->SetDampingCoeff(mMBDamping1, mMBDamping2);
    // mMultibody->SetAngleClamp(mMBEAngleClamp);
    mMultibody->SetMaxVel(mMBMaxVel);

    mMultibody->InitSimVars(mInternalWorld, mMBZeroInitPose, mMBZeroInitPoseVel,
                            true);

    // if (mMBEnableGuideAction == true)
    // {
    //     InitGuideTraj();
    // }

    // if (mMBTestJacobian)
    // {
    // 	mMultibody->TestJacobian();
    // 	mMultibody->TestSecondJacobian();
    // 	exit(0);
    // }
    // if (mEnablePDControl)
    // {
    // 	assert(mPDController != nullptr);
    // 	mPDController->Init(mMultibody);
    // 	// std::cout << "[debug] mPDController inited succ\n";
    // }
}

void btGeneralizeWorld::AddMultibody(cRobotModelDynamics *model)
{
    assert(model != nullptr);
    if (mMultibody == nullptr)
        delete mMultibody;
    mMultibody = model;
    // mMultibody->SetComputeSecondDerive(true);
    // mMultibody->SetDampingCoeff(mMBDamping1, mMBDamping2);
    // mMultibody->SetAngleClamp(mMBEAngleClamp);
    // mMultibody->SetMaxVel(mMBMaxVel);
}

cRobotModelDynamics *btGeneralizeWorld::GetMultibody() { return mMultibody; }
void btGeneralizeWorld::RemoveObj(int id)
{
    std::cout << "begin remove obj " << id << std::endl;
    // mInternalWorld->removeCollisionObject(mSimObjs[id]);
    delete mSimObjs[id];
    mSimObjs.erase(mSimObjs.begin() + id);
}

// extern const std::string rigidbody_path = "rigidbody.txt";
// extern const std::string multibody_path = "multiboody.txt";
// tVector before_angmom = tVector::Zero();
void OutputJacobianTestJson(cRobotModelDynamics *, const std::string &path);
void OutputDynamicsFTestJson(cRobotModelDynamics *, const std::string &path);
void btGeneralizeWorld::StepSimulation(double dt)
{
    // btTimeUtil::Begin("stepsim");
    // std::cout << "------------------begin step simulation for frame " <<
    // mFrameId << " time " << mTime << "------------------\n"; std::cout <<
    // "[bt] col obj num = " << mInternalWorld->getNumCollisionObjects() <<
    // std::endl; std::cout << "[bt update] q0 = " <<
    // mMultibody->Getq().transpose() << std::endl; btTimeUtil::Begin("step");
    if (enable_bullet_sim)
    {
        mInternalWorld->stepSimulation(dt);
    }
    else
    {
        ApplyGravity();
        CollisionDetect();
        UpdateAdviser(dt);
        // ApplyGuideAction();
        CollisionResponse(dt);
        // CheckGuideTraj();

        Update(dt);
        PostUpdate(dt);
    }

    mFrameId++;
    global_frame_id = mFrameId;
    // btTimeUtil::End("step");
    // if (mMultibody)
    // 	std::cout << "q = " << mMultibody->Getq().transpose() << std::endl;
    // btTimeUtil::End("stepsim");
}
void btGeneralizeWorld::PostUpdate(double dt) { mTime += dt; }
std::vector<btGenContactForce *> btGeneralizeWorld::GetContactForces() const
{
    return mContactForces;
}

btGenContactAwareAdviser *btGeneralizeWorld::GetContactAwareAdviser()
{
    return mControlAdviser;
}
std::vector<btPersistentManifold *>
btGeneralizeWorld::GetContactManifolds() const
{
    return mManifolds;
}

btDiscreteDynamicsWorld *btGeneralizeWorld::GetInternalWorld()
{
    return mInternalWorld;
}

btBroadphaseInterface *btGeneralizeWorld::GetBroadphase()
{
    return m_broadphase;
}
btCollisionDispatcher *btGeneralizeWorld::GetDispatcher()
{
    return m_dispatcher;
}
btDefaultCollisionConfiguration *btGeneralizeWorld::GetConfiguration()
{
    return m_collisionConfiguration;
}

btGeneralizeWorld::eContactResponseMode
btGeneralizeWorld::GetContactResponseMode() const
{
    return mContactMode;
}
#include "btGenUtil/BulletUtil.h"
void btGeneralizeWorld::createRigidBody(double mass,
                                        const btTransform &startTransform_,
                                        btCollisionShape *shape,
                                        const std::string &name,
                                        const btVector4 &color)
{
    // judge whether it is statid plane
    btTransform new_trans = startTransform_;
    {
        btStaticPlaneShape *static_plane =
            dynamic_cast<btStaticPlaneShape *>(shape);
        if (static_plane != nullptr)
        {
            // std::cout << "add static plane\n";
            btVector3 normal = static_plane->getPlaneNormal();
            btScalar constant = static_plane->getPlaneConstant();

            // recreate a feasible BIG cube
            mass = 0;
            double cube_size = 100;
            shape = createBoxShape(
                btVector3(cube_size / 2, cube_size / 2, cube_size / 2));
            tMatrix rot = btMathUtil::DirToRotMat(
                btBulletUtil::btVectorTotVector0(normal), tVector(0, 1, 0, 0));
            btVector3 translate = normal * (constant - cube_size / 2);
            // std::cout << "rot = \n"
            // 		  << rot << std::endl;
            // std::cout << "translate = " <<
            // btBulletUtil::btVectorTotVector0(translate).transpose() <<
            // std::endl;
            new_trans.setOrigin(translate);
            new_trans.setRotation(btBulletUtil::tQuaternionTobtQuaternion(
                btMathUtil::RotMatToQuaternion(rot)));
        }
    }

    btVector3 localInertia(0, 0, 0);

    if (true == std::isinf(mass))
    {
        std::cout << "inf mass is set to zero";
        mass = 0.0;
    }
    if (mass != 0.f)
        shape->calculateLocalInertia(mass, localInertia);

    btGenRigidBody::tParams params;
    params.col_shape = shape;
    params.damping = 0;
    params.mass = mass;
    params.name = name;
    params.local_inertia = btBulletUtil::btVectorTotVector0(localInertia);
    params.origin = btBulletUtil::btVectorTotVector1(new_trans.getOrigin());
    params.rot =
        btBulletUtil::btQuaternionTotQuaternion(new_trans.getRotation());

    btCollisionObject *obj = new btGenRigidBody(params);

    obj->setWorldTransform(new_trans);
    mSimObjs.push_back(dynamic_cast<btGenRigidBody *>(obj));

    obj->setCollisionShape(shape);
    obj->setUserPointer(nullptr);
    mInternalWorld->addCollisionObject(obj, 1, -1);
    // std::cout << "col flag = " << obj->getCollisionFlags() << std::endl;
    // exit(0);
}

// 1. Apply active force, update the velocity, write into bullet
void btGeneralizeWorld::ApplyTestActiveForce(double dt)
{
    // BT_PROFILE("apply_active_force");
    // add gravity to simobjs and multibody
    // std::cout << "total obj num = " << mSimObjs.size() << std::endl;

    // if (mMultibody)
    // 	if (mEnablePDControl)
    // 	{
    // 		std::cout << "prohibited\n";
    // 		exit(0);
    // 		mPDController->SetPDTargetq(tVectorXd::Zero(mMultibody->GetNumOfFreedom()));
    // 		mPDController->SetPDTargetqdot(tVectorXd::Zero(mMultibody->GetNumOfFreedom()));
    // 		mPDController->ApplyGeneralizedTau(dt);
    // 	}
    // std::cout << "mb pos = " << mMultibody->Getq().segment(0, 3).transpose()
    // << std::endl; std::cout << "[apply] q = " <<
    // mMultibody->Getq().transpose() << std::endl; std::cout << "qdot = " <<
    // mMultibody->Getqdot().transpose() << std::endl;

    if (mEnablePeturb)
    {
        if (mMultibody)
        {
            for (int i = 0; i < mMultibody->GetNumOfLinks(); i++)
            {
                mMultibody->ApplyJointTorque(i, tVector::Random() / 10);
                mMultibody->ApplyForce(std::rand() %
                                           mMultibody->GetNumOfLinks(),
                                       tVector::Random(), tVector::Random());
                mMultibody->ApplyLinkTorque(i, tVector::Random());
            }
            std::cout << "perturb Q = "
                      << mMultibody->GetGeneralizedForce().transpose()
                      << std::endl;
        }
    }
}

/**
 * \brief					Apply gravity
 */
void btGeneralizeWorld::ApplyGravity()
{
    for (auto &obj : mSimObjs)
        obj->ApplyCOMForce(obj->GetMass() * mGravity);

    if (mMultibody)
    {
        // std::cout << "[before gravity] multibody force = "
        //           << mMultibody->GetGeneralizedForce().transpose() <<
        //           std::endl;
        mMultibody->ApplyGravity(mGravity);
    }
}

/**
 * \brief				Update the contact-aware LCP adviser
 *
 */
void btGeneralizeWorld::UpdateAdviser(double dt)
{
    if (mMBEnableContactAwareLCP)
    {
        mControlAdviser->Update(dt);
    }
}

// 2. collision detect, collect contact info
void btGeneralizeWorld::CollisionDetect()
{
    mManifolds.clear();
    mInternalWorld->performDiscreteCollisionDetection();
    // std::cout << "[bt] dispatcher = " << m_dispatcher << std::endl;
    int num_of_manifolds = m_dispatcher->getNumManifolds();
    // std::cout << "[bt] num of manifolds = " << num_of_manifolds << std::endl;
    for (int i = 0; i < num_of_manifolds; i++)
        mManifolds.push_back(m_dispatcher->getManifoldByIndexInternal(i));

#ifdef __DEBUG__
    std::cout << "manifolds = " << num_of_manifolds << std::endl;
#endif
}

// 3. collision respose, calculate the contact force, update the velocity again
void btGeneralizeWorld::CollisionResponse(double dt)
{
    // no collision, return
    // if (mManifolds.size() == 0) return;
    // int num_contact_points = 0;
    // for(auto & x : mManifolds) num_contact_points += x->getNumContacts();
    // if (num_contact_points == 0) return;

    // here, we must record the pre-collsiion state if the convert_mat_test is
    // enabled in the LCPSolver.

    // PushStatePreCollision();

    // update velocity by active force now
    // if (false == mMBUpdateVelWithoutCoriolis)
    // {
    // 	UpdateVelocityInternal(dt);
    // }
    // else
    // {
    // 	UpdateVelocityInternalWithoutCoriolis(dt);
    // }

    // // because "UpdateVelocity" is needed in the LCP test function as well,
    // so we need to clear active force here as well ClearForce();

    // std::cout << "[before lcp] q = " << mMultibody->Getq().transpose() <<
    // std::endl;

    // std::cout << "[before lcp] qdot = " << mMultibody->Getqdot().transpose()
    // << std::endl; exit(0); tVectorXd qold = mMultibody->Getq(); tVectorXd
    // qdotold = mMultibody->Getqdot(); tVectorXd qddotold =
    // mMultibody->Getqddot();
    // 4. collision response
    // btTimeUtil::Begin("LCP total");
    switch (mContactMode)
    {
    case eContactResponseMode::PenaltyMode:
        CollisionResponsePenalty(dt);
        break;
    case eContactResponseMode::LCPMode:
        CollisionResponseLCP(dt);
        break;
    case eContactResponseMode::SequentialImpulseMode:
        std::cout << "[error] Sequential Impulse Mode has been depracated\n";
        exit(0);
        break;
    case eContactResponseMode::NoMode:
        std::cout << "[log] No Contact Response\n";
        break;
    default:
        std::cout << "[error] No contact mode " << mContactMode << std::endl;
        exit(0);
        break;
    }
    // btTimeUtil::End("LCP total");
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
    // std::cout << "[bt] num of contact forces = " << mContactForces.size() <<
    // std::endl;
    for (auto &f : mContactForces)
    {
        // std::cout << "add contact f = " << f->mForce.transpose() <<
        // std::endl;
        f->mObj->ApplyForce(f->mForce, f->mWorldPos);
    }

    // apply contact torques
    for (auto &t : mConstraintGenalizedForce)
    {
        // std::cout << "[sim] model dof " << t->dof_id << " add constraint
        // generalized force " << t->value << std::endl;
        // t->model->ApplyJointTorque(t->joint_id, t->joint_torque);
        t->model->ApplyGeneralizedForce(t->dof_id, t->value);
    }
    // std::cout << "[btGenWorld] frame " << global_frame_id << " q = " <<
    // mMultibody->Getq().transpose() << std::endl; std::cout << "[btGenWorld]
    // frame " << global_frame_id << " qdot = " <<
    // mMultibody->Getqdot().transpose() << std::endl; std::cout <<
    // "[btGenWorld] frame " << global_frame_id << " qddot = " <<
    // mMultibody->Getqddot().transpose() << std::endl; std::cout <<
    // "[btGenWorld] frame " << global_frame_id << " M = \n"
    // 		  << mMultibody->GetMassMatrix() << std::endl;
    // std::cout << "[btGenWorld] frame " << global_frame_id << " C = \n"
    // 		  << mMultibody->GetCoriolisMatrix() << std::endl;

    // std::cout << "[bt] after constraint, Q = " <<
    // mMultibody->GetGeneralizedForce().transpose() << std::endl; std::ofstream
    // fout(log_path, std::ios::app); fout << "------frame " << global_frame_id
    // << "------\n"; fout << "q = " << mMultibody->Getq().transpose() <<
    // std::endl; fout << "qdot = " << mMultibody->Getqdot().transpose() <<
    // std::endl; fout << "Q = " <<
    // mMultibody->GetGeneralizedForce().transpose() << std::endl; fout.close();
}

extern btGenRigidBody *UpcastRigidBody(const btCollisionObject *col);
extern btGenRobotCollider *UpcastRobotCollider(const btCollisionObject *col);

void btGeneralizeWorld::CollisionResponsePenalty(double dt)
{
    const double k = 2000;
    const double penetraction_threshold = 1e-3;
    for (auto &m : mManifolds)
    {
        // 1. get the collision pair
        int num_of_contacts = m->getNumContacts();
        btGenRigidBody *rigidbody0 = UpcastRigidBody(m->getBody0()),
                       *rigidbody1 = UpcastRigidBody(m->getBody1());
        btGenRobotCollider *mbody0 = nullptr, *mbody1 = nullptr;

        if (rigidbody0 == nullptr)
            mbody0 = UpcastRobotCollider(m->getBody0()),
            assert(mbody0 != nullptr);
        if (rigidbody1 == nullptr)
            mbody1 = UpcastRobotCollider(m->getBody1()),
            assert(mbody1 != nullptr);

        for (int i = 0; i < num_of_contacts; i++)
        {
            // 2. determine the contact normal on body0
            auto &pt = m->getContactPoint(i);

            tVector normal_on_0 =
                btBulletUtil::btVectorTotVector0(pt.m_normalWorldOnB);
            // std::cout << "body0 name = " << body0->GetName() << std::endl;

            // 3. penetration
            double penetraction = penetraction_threshold - pt.getDistance();
            // std::cout << "pene = " << penetraction << std::endl;

            // 4. apply the contact force
            tVector force_on_0 = penetraction * k * normal_on_0;
            // std::cout << "force on 0 = " << force_on_0.transpose() <<
            // std::endl;

            if (rigidbody0)
                rigidbody0->ApplyForce(
                    force_on_0,
                    btBulletUtil::btVectorTotVector1(pt.m_positionWorldOnA));
            else
                mbody0->mModel->ApplyForce(
                    mbody0->mLinkId, force_on_0,
                    btBulletUtil::btVectorTotVector0(pt.m_positionWorldOnA));

            if (rigidbody1)
                rigidbody1->ApplyForce(
                    -force_on_0,
                    btBulletUtil::btVectorTotVector1(pt.m_positionWorldOnB));
            else
                mbody1->mModel->ApplyForce(
                    mbody1->mLinkId, -force_on_0,
                    btBulletUtil::btVectorTotVector0(pt.m_positionWorldOnB));

            // body1->ApplyForce(-force_on_0,
            // btBulletUtil::btVectorTotVector1(pt.m_positionWorldOnB) -
            // body1->GetWorldPos()); exit(0);
        }
    }

    UpdateVelocityInternal(dt);
}

/**
 * \brief					Compute constraint forces by LCP
 * formulaes
 */
void btGeneralizeWorld::CollisionResponseLCP(double dt)
{
    // mMultibody->PushState("second");
    mLCPContactSolver->ConstraintProcess(dt);
    mContactForces = mLCPContactSolver->GetContactForces();
    mConstraintGenalizedForce =
        mLCPContactSolver->GetConstraintGeneralizedForces();
    // mMultibody->PopState("second");
    // now the first contact-aware LCP has been solved, we get the control force
    // and contact force but it is not consistent with the normal LCP, so we
    // needs to apply this control force, and then solve this LCP again to get a
    // "normal" contact force, then record, then if (mMBEnableContactAwareLCP ==
    // true)
    // {
    // 	// std::cout << "begin to solve LCP for the second time\n";
    // 	// mMultibody->SetEnableContactAwareAdviser(false);

    // 	// for (auto& x : mContactForces)
    // 	// {
    // 	// 	if (x->mObj->GetType() == eColObjType::RobotCollder)
    // 	// 	{
    // 	// 		std::cout << "[old] contact force = " <<
    // x->mForce.transpose()
    // << std::endl;
    // 	// 	}
    // 	// }
    // 	// apply active control force
    // 	// mMultibody->PushState("second");
    // 	// for (auto& t : mConstraintGenalizedForce)
    // 	// {
    // 	// 	t->model->ApplyGeneralizedForce(t->dof_id, t->value);
    // 	// }
    // 	// mLCPContactSolver->ConstraintProcess(dt);
    // 	// mMultibody->PopState("second");
    // 	// mContactForces = mLCPContactSolver->GetContactForces();
    // 	// for (auto& x : mContactForces)
    // 	// {
    // 	// 	if (x->mObj->GetType() == eColObjType::RobotCollder)
    // 	// 	{
    // 	// 		std::cout << "[new] contact force = " <<
    // x->mForce.transpose()
    // << std::endl;
    // 	// 	}
    // 	// }
    // 	// mMultibody->SetEnableContactAwareAdviser(true);
    // }

    // std::cout << "[btGenworld] frame " << global_frame_id << std::endl;
    // for (int i = 0; i < mContactForces.size(); i++)
    // {
    // 	auto obj = mContactForces[i]->mObj;
    // 	auto multibody_collider = UpcastRobotCollider(obj);
    // 	if (multibody_collider != nullptr)
    // 	{
    // 		std::cout << "[btGenworld] contact " << i << " force = " <<
    // mContactForces[i]->mForce.transpose() << std::endl; std::cout
    // <<
    // "[btGenworld] contact " << i << " pos = " <<
    // mContactForces[i]->mWorldPos.transpose() << std::endl; tMatrixXd jac;

    // 		mMultibody->ComputeJacobiByGivenPointTotalDOFWorldFrame(multibody_collider->mLinkId,
    // mContactForces[i]->mWorldPos.segment(0, 3), jac); std::cout
    // <<
    // "[btGenworld] contact " << i << " jac = \n"
    // 				  << jac << std::endl;
    // 		std::cout << "[btGenworld] contact " << i << " Q = "
    // 				  << (jac.transpose() *
    // mContactForces[i]->mForce.segment(0, 3)).transpose() << std::endl;
    // 	}
    // }
    // if (mContactTorques.size()) std::cout << "[sim] get constraint torque 0 =
    // " << mContactTorques[0]->joint_torque.transpose() << std::endl; exit(0);
}

void btGeneralizeWorld::CollisionResposeSI(double dt) {}

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
    for (auto &obj : mSimObjs)
        obj->UpdateVelocity(dt);
    if (mMultibody)
    {
        mMultibody->UpdateVelocity(dt);
    }
}

void btGeneralizeWorld::UpdateVelocityInternalWithoutCoriolis(double dt)
{
    for (auto &obj : mSimObjs)
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
        if (mFrameId < mMBCollectFrameNum)
            CollectFrameInfo(dt);
        if (mFrameId == mMBCollectFrameNum)
        {
            // gPauseSimulation = true;
            WriteFrameInfo("frame_info.txt");
        }
    }

    // for rigidbodies
    for (auto &obj : mSimObjs)
    {
        obj->UpdateVelocity(dt);
        obj->UpdateTransform(dt);
        obj->WriteTransAndVelToBullet();
    }

    // for multibody
    if (mMultibody)
    {
        // if (mMBEnableRk4)
        // {
        // 	mMultibody->UpdateRK4(dt);
        // }
        // else
        // {
        // 	// std::cout << "[bt] old q = " <<
        // mMultibody->Getq().transpose() << std::endl;
        // 	// std::cout << "[bt] old qdot = " <<
        // mMultibody->Getqdot().transpose() << std::endl;

        // 	// std::cout << "[bt] new q = " <<
        // mMultibody->Getq().transpose() << std::endl;
        // 	// std::cout << "[bt] new qdot = " <<
        // mMultibody->Getqdot().transpose() << std::endl;
        // }
        // std::cout << "[preupdate] q = " << mMultibody->Getq().transpose() <<
        // std::endl; std::cout << "[preupdate] qdot = " <<
        // mMultibody->Getqdot().transpose() << std::endl; std::cout <<
        // "[preupdate] gen force = " <<
        // mMultibody->GetGeneralizedForce().transpose() << std::endl; std::cout
        // << "[preupdate] qddot = " << mMultibody->Getqddot().transpose() <<
        // std::endl; std::cout << "[preupdate] M = " <<
        // mMultibody->GetMassMatrix() << std::endl; std::cout << "[preupdate] C
        // = " << mMultibody->GetCoriolisMatrix() << std::endl;
        // {
        // 	std::cout << "two time update\n";
        // 	if (mMBEnableContactAwareLCP == false)
        // 	{
        // 		mMultibody->UpdateVelocity(dt);
        // 	}
        // 	else
        // 	{
        // 		mMultibody->GetContactAwareAdviser()->UpdateMultibodyVelocityDebug(dt);
        // 	}
        // 	mMultibody->UpdateTransform(dt);
        // }

        {
            // std::cout << "one time update\n";
            if (mMBEnableContactAwareLCP == false)
            {
                mMultibody->UpdateVelocityAndTransform(dt);
            }
            else
            {
                mMultibody->GetContactAwareAdviser()
                    ->UpdateMultibodyVelocityAndTransformDebug(dt);
            }
            // mMultibody->UpdateTransform(dt);
        }

        if (mMultibody->IsGeneralizedMaxVel())
        {
            std::cout << "[warn] multibody exceed max vel = "
                      << mMultibody->Getqdot().cwiseAbs().maxCoeff()
                      << std::endl;
            // if (mEnablePauseWhenMaxVel) gPauseSimulation = true;
            // exit(0);
        }
    }
}

void btGeneralizeWorld::ClearForce()
{
    for (auto &obj : mSimObjs)
    {
        obj->ClearForce();
    }

    if (mMultibody)
    {
        mMultibody->ClearForce();
    }
}

btBoxShape *btGeneralizeWorld::createBoxShape(const btVector3 &halfExtents)
{
    btBoxShape *box = new btBoxShape(halfExtents);
    mCollisionShapeArray.push_back(box);
    return box;
}

void btGeneralizeWorld::PushStatePreCollision()
{
    // std::cout << "here, only push & pop velocity and force state\n";
    std::string name = "pre_collision";
    for (auto &obj : mSimObjs)
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
    for (auto &obj : mSimObjs)
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
    info.frame_id = mFrameId;
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

void btGeneralizeWorld::WriteFrameInfo(const std::string &path)
{
    Json::Value root;
    std::ofstream fout(path);
    root["link_num"] = mMultibody->GetNumOfLinks();
    root["dof_num"] = mMultibody->GetNumOfFreedom();
    root["frame_num"] = mFrameInfo.size();
    Json::Value FrameArray = Json::arrayValue;
    for (auto &item : mFrameInfo)
    {
        Json::Value FrameValue;
        FrameValue["frame_id"] = item.frame_id;
        FrameValue["q"] = btJsonUtil::BuildVectorJsonValue(item.q);
        FrameValue["qdot"] = btJsonUtil::BuildVectorJsonValue(item.qdot);
        FrameValue["qddot"] = btJsonUtil::BuildVectorJsonValue(item.qddot);
        FrameValue["Q"] = btJsonUtil::BuildVectorJsonValue(item.Q);
        FrameValue["residual"] =
            btJsonUtil::BuildVectorJsonValue(item.residual);
        FrameValue["timestep"] = item.timestep;

        for (int i = 0; i < mMultibody->GetNumOfLinks(); i++)
        {
            FrameValue["force_array"].append(
                btJsonUtil::BuildVectorJsonValue(item.force_array[i]));
            FrameValue["torque_array"].append(
                btJsonUtil::BuildVectorJsonValue(item.torque_array[i]));
        }
        FrameValue["mass_mat"] =
            btJsonUtil::BuildMatrixJsonValue(item.mass_mat);
        FrameValue["mass_mat_inv"] =
            btJsonUtil::BuildMatrixJsonValue(item.mass_mat_inv);
        FrameValue["coriolis_mat"] =
            btJsonUtil::BuildMatrixJsonValue(item.coriolis_mat);
        FrameValue["damping_mat"] =
            btJsonUtil::BuildMatrixJsonValue(item.damping_mat);
        FrameArray.append(FrameValue);
    }
    root["frame_array"] = FrameArray;
    btJsonUtil::WriteJson(path, root, true);
    fout.close();
}

void OutputJacobianTestJson(cRobotModelDynamics *mb, const std::string &path)
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

        link_json["local_pos"] = btJsonUtil::BuildVectorJsonValue(local_pos);
        link_json["global_pos"] = btJsonUtil::BuildVectorJsonValue(global_pos);
        mb->ComputeJacobiByGivenPointTotalDOFWorldFrame(
            i, global_pos.segment(0, 3), jac_buf);
        link_json["jacobian"] = btJsonUtil::BuildMatrixJsonValue(jac_buf);
        link_json["world_trans"] =
            btJsonUtil::BuildMatrixJsonValue(link->GetGlobalTransform());
        root["link_" + std::to_string(i)] = link_json;
    }

    btJsonUtil::WriteJson(path.c_str(), root, true);
}

void OutputDynamicsFTestJson(cRobotModelDynamics *mb, const std::string &path)
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

        link_json["force_local_pos"] =
            btJsonUtil::BuildVectorJsonValue(local_pos);
        link_json["force_global_pos"] =
            btJsonUtil::BuildVectorJsonValue(global_pos);
        link_json["force_global"] =
            btJsonUtil::BuildVectorJsonValue(global_force.segment(0, 3));
        mb->ApplyForce(i, global_force, global_pos);

        // link_json["jacobian"] = cJsonUtil::BuildMatrixJsonValue(jac_buf);
        root["link_" + std::to_string(i)] = link_json;
    }
    root["generalized_force"] =
        btJsonUtil::BuildVectorJsonValue(mb->GetGeneralizedForce());

    root["mass_mat"] = btJsonUtil::BuildMatrixJsonValue(mb->GetMassMatrix());
    root["coriolis_mat"] =
        btJsonUtil::BuildMatrixJsonValue(mb->GetCoriolisMatrix());

    tVectorXd qddot =
        mb->GetInvMassMatrix() *
        (mb->GetGeneralizedForce() - mb->GetCoriolisMatrix() * mb->Getqdot());
    root["qddot"] = btJsonUtil::BuildVectorJsonValue(qddot);
    btJsonUtil::WriteJson(path.c_str(), root, true);
}

/**
 * \brief			Used in order to clear the whold world after
 * each episode (DeepMimic)
 */
void btGeneralizeWorld::Reset()
{
    mTime = 0;
    mFrameId = 0;

    global_frame_id = mFrameId;
    mMultibody->ClearForce();
    mContactForces.clear();

    m_broadphase->resetPool(m_dispatcher);

    btOverlappingPairCache *pair_cache =
        m_broadphase->getOverlappingPairCache();
    btBroadphasePairArray &pair_array = pair_cache->getOverlappingPairArray();
    for (int i = 0; i < pair_array.size(); ++i)
    {
        pair_cache->cleanOverlappingPair(pair_array[i], m_dispatcher);
    }
    mMultibody->SyncToBullet();
}

void btGeneralizeWorld::SetEnableContacrAwareControl()
{
    std::cout << "btGeneralizeWorld::SetEnableContacrAwareControl is called\n";
    if (mControlAdviser != nullptr || mMBEnableContactAwareLCP == true)
    {
        std::cout << "[error] control adviser exists, exit...\n";
        exit(0);
    }
    mMBEnableContactAwareLCP = true;
    mControlAdviser = new btGenContactAwareAdviser(this);
    mControlAdviser->Init(this->mMultibody, mContactAwareConfig);
}

// /**
//  * \brief			Initialize the guide trajectory from target file
// */
// void btGeneralizeWorld::InitGuideTraj()
// {
// 	if (mMBEnableGuideAction == true)
// 	{
// 		// const int max_frame = 100;
// 		mGuideTraj = new btTraj();

// 		mGuideTraj->LoadTraj(mGuidedActionTrajFile, mMultibody);

// 		mFrameId = 1;
// 		mMultibody->SetqAndqdot(mGuideTraj->mq[mFrameId],
// mGuideTraj->mqdot[mFrameId]);
// 		// std::cout << "init guide traj done for " <<
// mGuidedActionTrajFile << std::endl;
// 		// exit(0);
// 	}
// }

// /**
//  * \brief			Apply active action
// */
// void btGeneralizeWorld::ApplyGuideAction()
// {
// 	if (mMBEnableGuideAction == true)
// 	{
// 		if (mFrameId >= mGuideTraj->mNumOfFrames - 1)
// 		{
// 			std::cout << "guided traj exceed, exit\n";
// 			exit(0);
// 		}
// 		tVectorXd force = mGuideTraj->mActiveForce[mFrameId];
// 		std::cout << "cur frame id " << mFrameId << " active force = "
// << force.transpose() << std::endl; 		tVectorXd ref_q =
// mGuideTraj->mq[mFrameId], 				  now_q =
// this->mMultibody->Getq(); 		tVectorXd ref_qdot =
// mGuideTraj->mqdot[mFrameId], 				  now_qdot =
// mMultibody->Getqdot(); 		tVectorXd q_diff = ref_q - now_q;
// tVectorXd qdot_diff
// = ref_qdot - now_qdot; 		std::cout << "q diff norm = " <<
// q_diff.norm() << std::endl; 		std::cout << "qdot diff norm = " <<
// qdot_diff.norm() << std::endl; if (q_diff.norm() < 1e-8 && qdot_diff.norm() <
// 1e-6)
// 		{
// 			std::cout << "set q and qdot from the ref motion\n";
// 			mMultibody->SetqAndqdot(ref_q, ref_qdot);
// 		}
// 		for (int dof = 0; dof < mMultibody->GetNumOfFreedom(); dof++)
// 		{
// 			mMultibody->ApplyGeneralizedForce(dof, force[dof]);
// 		}
// 	}
// }

// void btGeneralizeWorld::CheckGuideTraj()
// {
// 	if (mMBEnableGuideAction == true)
// 	{
// 		for (auto& x : mContactForces)
// 		{
// 			if (x->mObj->GetType() == eColObjType::RobotCollder)
// 			{
// 				std::cout << "[cur] link " <<
// dynamic_cast<btGenRobotCollider*>(x->mObj)->mLinkId << " force = " <<
// x->mForce.transpose() << " pos = " << x->mWorldPos.transpose() << std::endl;
// 			}
// 		}

// 		for (int id = 0; id <
// mGuideTraj->mContactForce[mFrameId].size(); id++)
// 		{
// 			auto x = mGuideTraj->mContactForce[mFrameId][id];
// 			if (x->mObj->GetType() == eColObjType::RobotCollder)
// 			{
// 				std::cout << "[ref] link " <<
// dynamic_cast<btGenRobotCollider*>(x->mObj)->mLinkId << " force = " <<
// x->mForce.transpose() << " pos = " << x->mWorldPos.transpose() << std::endl;
// 			}
// 		}
// 	}
// }