#include "btGenWorld.h"
#include "BulletGenDynamics/btGenCollision/btGenCollisionDispatcher.h"
#include "BulletGenDynamics/btGenController/BuildController.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/Trajectory/btTraj.h"
#include "BulletGenDynamics/btGenModel/EulerAngelRotationMatrix.h"
#include "BulletGenDynamics/btGenUtil/TimeUtil.hpp"
#include "btGenController/ContactAwareController/btGenContactAwareController.h"
#include "btGenModel/RobotModelDynamics.h"
#include "btGenModel/SimObj.h"
#include "btGenSolver/ContactManager.h"
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
    mGround = nullptr;
    mGuideTraj = nullptr;
    mContactManager = nullptr;
    mFrameId = 0;
    mIntegrationScheme = eIntegrationScheme::OLD_SEMI_IMPLICIT_SCHEME;
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
            for (int i = 0; i < mMultibody->GetNumOfLinks(); i++)
            {
                mInternalWorld->removeCollisionObject(
                    mMultibody->GetLinkCollider(i));
            }
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
    delete mCtrl;
    delete mContactManager;
}

void btGeneralizeWorld::Init(const std::string &config_path)
{
    Json::Value dispatcher_conf;
    {
        Json::Value config_js;
        btJsonUtil::LoadJson(config_path, config_js);
        dispatcher_conf = config_js;
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
            mEnablePeturb =
                btJsonUtil::ParseAsDouble("enable_perturb", config_js);
            mLCPConfigPath =
                btJsonUtil::ParseAsString("lcp_config_path", config_js);
            mIntegrationScheme = BuildIntegrationScheme(
                btJsonUtil::ParseAsString("integration_scheme", config_js));
        }

        mLCPContactSolver = nullptr;
        mManifolds.clear();

        mFrameInfo.clear();
    }

    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btGenCollisionDispatcher(dispatcher_conf, this,

                                                m_collisionConfiguration);
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
        mLCPContactSolver = new btGenContactSolver(mLCPConfigPath, this);
    }

    // if (mEnablePDControl)
    // {
    // 	mPDController = new btGenPDController(mPDControllerPath);
    // }
    mCtrl = nullptr;
    mContactManager = new btGenContactManager(this);
    // mControlController = new btGenContactAwareController();
    // SetEnableContacrAwareControl();
}

void btGeneralizeWorld::AddGround(double height)
{
    if (mGround == nullptr)
    {
        btStaticPlaneShape *plane =
            new btStaticPlaneShape(btVector3(0, 1, 0), height);
        mCollisionShapeArray.push_back(plane);
        btTransform trans;
        trans.setIdentity();
        trans.setOrigin(btVector3(0, 0, 0));
        createRigidBody(0, trans, plane, "ground");
        mGround = mSimObjs[mSimObjs.size() - 1];
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

const btCollisionObject *btGeneralizeWorld ::GetGround() const
{
    return this->mGround;
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

    mMultibody->InitSimVars(this, mMBZeroInitPose, mMBZeroInitPoseVel, true);
    this->m_dispatcher->SetModel(mMultibody);

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
    // std::cout << "------------------begin step simulation for frame "
    //           << mFrameId << " time " << mTime << "------------------\n";
    // std::cout <<
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
        UpdateController(dt);
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

btGenControllerBase *btGeneralizeWorld::GetController() { return mCtrl; }
btGenContactAwareController *btGeneralizeWorld::GetContactAwareController()
{
    if (HasContactAwareController() == false)
    {
        printf("[error] GetContactAwareController cannot be called because "
               "there is no consistent controller\n");
        exit(0);
        return nullptr;
    }
    return dynamic_cast<btGenContactAwareController *>(mCtrl);
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
btGenCollisionDispatcher *btGeneralizeWorld::GetDispatcher()
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
 * \brief				Update the contact-aware LCP controller
 *
 */
void btGeneralizeWorld::UpdateController(double dt)
{
    if (HasController() == true)
        mCtrl->Update(dt);
}

// 2. collision detect, collect contact info
void btGeneralizeWorld::CollisionDetect()
{
    mManifolds.clear();
    mInternalWorld->performDiscreteCollisionDetection();
    m_dispatcher->Update();
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

    // if (mMBEnableContactAwareLCP == true && mContactForces.size() != 0 &&
    //     mDebugThreeContactForces == true)
    // {
    //     std::cout << "[debug] begin to debug three contact forces\n";
    //     // 1. save the contact force and control force

    //     std::vector<btGenContactForce *> mContactForces_old(0);
    //     for (auto &x : mContactForces)
    //         mContactForces_old.push_back(new btGenContactForce(*x));

    //     std::vector<btGenConstraintGeneralizedForce *>
    //         mConstraintGenalizedForce_old(0);
    //     for (auto &x : mConstraintGenalizedForce)
    //         mConstraintGenalizedForce_old.push_back(
    //             new btGenConstraintGeneralizedForce(*x));

    //     std::vector<btGenConstraintGeneralizedForce *>
    //         mContactAwareControlForce_old(0);

    //     for (auto &x : mContactAwareControlForce)
    //         mContactAwareControlForce_old.push_back(
    //             new btGenConstraintGeneralizedForce(*x));

    //     // 2. apply the control force, resolve the contact force
    //     // 3. compare the contact force
    //     std::cout
    //         << "------------------contact aware contact forces-------------\n";
    //     DebugPrintContactForce(mContactForces);

    //     // CollisionResponseLCP(dt);
    //     auto active_applied_contact_forces =
    //         DebugGetContactForces(dt, mContactAwareControlForce);
    //     std::cout
    //         << "------------------active applied contact forces-------------\n";
    //     std::cout << "now apply force = ";
    //     for (auto &x : mContactAwareControlForce)
    //     {
    //         std::cout << x->value << " ";
    //     }
    //     std::cout << std::endl;
    //     DebugPrintContactForce(active_applied_contact_forces);

    //     // output the legacy contact forces
    //     std::cout
    //         << "------------------contact forces from ref traj-------------\n";
    //     int ref_frameid = mControlController->GetRefFrameId() - 1;
    //     // std::cout << "ref frame id = " << ref_frameid << std::endl;
    //     auto traj = mControlController->GetRefTraj();
    //     DebugPrintContactForce(traj->mContactForce[ref_frameid]);
    //     tVectorXd legacy_gen_contact_force =
    //         DebugConvertCartesianContactForceToGenForce(
    //             traj->mq[ref_frameid], traj->mContactForce[ref_frameid]);
    //     // std::cout << "gen = " << legacy_gen_contact_force.transpose()
    //     //           << std::endl;

    //     tVectorXd legacy_active_force = DebugGetGenControlForce(ref_frameid);
    //     std::cout << "truth joint forces = " << legacy_active_force.transpose()
    //               << std::endl;

    //     std::cout << "-------------contact force after applying the control "
    //                  "force from ref traj--------------\n";
    //     auto contact_forces_after_applying_the_ref_control =
    //         DebugGetContactForces(dt, legacy_active_force);
    //     DebugPrintContactForce(contact_forces_after_applying_the_ref_control);

    //     std::cout << "q = " << mMultibody->Getq().norm() << std::endl;
    //     std::cout << "qdot = " << mMultibody->Getqdot().norm() << std::endl;
    //     // 4. restore the controller state, and the old contact forces
    //     this->mMultibody->SetEnableContactAwareController(true);
    //     mMBEnableContactAwareLCP = true;

    //     // apply the old contact forces, then return
    //     mContactForces = mContactForces_old;
    //     mConstraintGenalizedForce = mConstraintGenalizedForce_old;
    //     mContactAwareControlForce = mContactAwareControlForce_old;
    // }
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

    // apply contact aware control force
    for (auto &t : mContactAwareControlForce)
    {
        t->model->ApplyGeneralizedForce(t->dof_id, t->value);
    }
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
    // std::cout << "[bt] contact forces num = " << mContactForces.size()
    //           << std::endl;
    mConstraintGenalizedForce =
        mLCPContactSolver->GetConstraintGeneralizedForces();

    // restore the active force when contact-aware LCP is enabled

    if (mMultibody != nullptr &&
        mMultibody->GetEnableContactAwareController() == true)
    {
        for (auto &x : mContactAwareControlForce)
            delete x;
        mContactAwareControlForce.clear();
        // std::cout << "[log] restore active force by contact-aware LCP is
        // enabled\n";
        auto model = mMultibody;
        auto controller = model->GetContactAwareController();
        // tMatrixXd E = controller->GetE();
        // tMatrixXd Dinv = controller->GetD().inverse();
        // tVectorXd b = controller->Getb();
        int num_of_freedom = model->GetNumOfFreedom();
        tVectorXd Q = tVectorXd::Zero(num_of_freedom);
        for (auto &force : mContactForces)
        {
            if (eColObjType::RobotCollder == force->mObj->GetType())
            {
                auto collider = static_cast<btGenRobotCollider *>(force->mObj);
                int link_id = collider->mLinkId;
                if (model != collider->mModel)
                {
                    std::cout
                        << "[error] restore active force model inconsistent\n";
                    exit(0);
                }

                // get the jacobian
                tMatrixXd jac;
                model->ComputeJacobiByGivenPointTotalDOFWorldFrame(
                    link_id, force->mWorldPos.segment(0, 3), jac);
                tVectorXd Q_single =
                    jac.transpose() * force->mForce.segment(0, 3);
                // std::cout << "Q_single = " << Q_single.transpose() <<
                // std::endl;
                Q += Q_single;
            }
        }
        tVectorXd control_force_underactuated = controller->CalcControlForce(Q);
        for (int dof = 6; dof < num_of_freedom; dof++)
        {
            mContactAwareControlForce.push_back(
                new btGenConstraintGeneralizedForce(
                    model, dof, control_force_underactuated[dof - 6]));
        }
    }
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

// void btGeneralizeWorld::UpdateVelocityInternalWithoutCoriolis(double dt)
// {
//     for (auto &obj : mSimObjs)
//         obj->UpdateVelocity(dt);
//     if (mMultibody)
//     {
//         mMultibody->UpdateVelocityWithoutCoriolis(dt);
//     }
// }

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

        if (HasContactAwareController() == false)
        {
            mMultibody->UpdateVelocityAndTransform(dt);
        }
        else
        {
            mMultibody->GetContactAwareController()
                ->UpdateMultibodyVelocityAndTransformDebug(dt);
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

// /**
//  * \brief               Given the control force, calculate the current contact forces
//  *  Note that these contact forces are calculated in non-contact-aware environmentI
// */
// std::vector<btGenContactForce *> btGeneralizeWorld::DebugGetContactForces(
//     double dt, const std::vector<btGenConstraintGeneralizedForce *> &gen_forces)
// {
//     // 2. save the current model state, save the env setting (contact aware or not)
//     bool world_enable_contact_aware = mMBEnableContactAwareLCP;
//     bool model_enable_contact_aware =
//         mMultibody->GetEnableContactAwareController();
//     mMultibody->PushState("debug_get_contact_force");
//     // 3. apply the control force
//     for (auto &f : gen_forces)
//     {
//         mMultibody->ApplyGeneralizedForce(f->dof_id, f->value);
//     }

//     // 4. change the env to non-contact-aware environment, collect the reaction contact force,
//     mMultibody->SetEnableContactAwareController(false);
//     mMBEnableContactAwareLCP = false;

//     // 5. record the reaction contact force
//     mLCPContactSolver->ConstraintProcess(dt);
//     std::vector<btGenContactForce *> contact_forces =
//         mLCPContactSolver->GetContactForces();

//     // 6. resotre the model state, and the env setting
//     mMultibody->PopState("debug_get_contact_force");
//     mMultibody->SetEnableContactAwareController(model_enable_contact_aware);
//     mMBEnableContactAwareLCP = world_enable_contact_aware;
//     // 7. return the contact forces
//     return contact_forces;
// }

// std::vector<btGenContactForce *>
// btGeneralizeWorld::DebugGetContactForces(double dt,
//                                          const tVectorXd &control_force)
// {
//     // 1. make sure the length of control for = actuated dof
//     int num_of_freedom = mMultibody->GetNumOfFreedom();
//     int num_of_actuated_freedom = num_of_freedom - 6;

//     if (control_force.size() != num_of_actuated_freedom)
//     {
//         std::cout << "[error] DebugGetContactForces: control force length "
//                      "inconsistent "
//                   << control_force << " != " << num_of_actuated_freedom
//                   << std::endl;
//         exit(0);
//     }

//     // 1. convert the gen forces to a single control force vector
//     std::vector<btGenConstraintGeneralizedForce *> gen_forces(0);
//     for (int i = 0; i < control_force.size(); i++)
//     {
//         gen_forces.push_back(new btGenConstraintGeneralizedForce(
//             mMultibody, i + 6, control_force[i]));
//     }

//     // 2. call another API, return
//     return DebugGetContactForces(dt, gen_forces);
// }

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
    info.qddot = mMultibody->Getqddot(dt);
    mFrameInfo.push_back(info);
}

void btGeneralizeWorld::WriteFrameInfo(const std::string &path)
{
    Json::Value root;
    std::ofstream fout(path);
    root["link_num"] = mMultibody->GetNumOfLinks();
    root["dof_num"] = mMultibody->GetNumOfFreedom();
    root["frame_num"] = static_cast<int>(mFrameInfo.size());
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

bool btGeneralizeWorld::HasController() const { return mCtrl != nullptr; }
bool btGeneralizeWorld::HasContactAwareController() const
{
    return this->HasController() &&
           mCtrl->GetCtrlType() == ebtGenControllerType::ContactAwareController;
}

btGenContactManager *btGeneralizeWorld::GetContactManager() const
{
    return mContactManager;
}

btGeneralizeWorld::eIntegrationScheme
btGeneralizeWorld::BuildIntegrationScheme(const std::string str)
{
    if (str == "old_semi_implicit")
    {
        return eIntegrationScheme::OLD_SEMI_IMPLICIT_SCHEME;
    }
    else if (str == "new_semi_implicit")
    {
        return eIntegrationScheme::NEW_SEMI_IMPLICIT_SCHEME;
    }
    else
    {
        BTGEN_ASSERT(false);
        return eIntegrationScheme::OLD_SEMI_IMPLICIT_SCHEME;
    }
}

std::string btGeneralizeWorld::BuildIntegrationSchemeStr(
    btGeneralizeWorld::eIntegrationScheme scheme)
{
    switch (scheme)
    {
    case btGeneralizeWorld::eIntegrationScheme::NEW_SEMI_IMPLICIT_SCHEME:
    {
        return "new_semi_implicit";
        break;
    }
    case btGeneralizeWorld::eIntegrationScheme::OLD_SEMI_IMPLICIT_SCHEME:
    {
        return "old_semi_implicit";
        break;
    }
    default:
        BTGEN_ASSERT(false);
        break;
    }
    return "";
}
void btGeneralizeWorld::AddController(const std::string &path)
{
    // 1. build & init the controller
    mCtrl = BuildController(this, path);
    mCtrl->Init(mMultibody, path);
    m_dispatcher->SetController(mCtrl);
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
// 		tVectorXd force = mGuideTraj->mAction[mFrameId];
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
btGeneralizeWorld::eIntegrationScheme
btGeneralizeWorld::GetIntegrationScheme() const
{
    return mIntegrationScheme;
}