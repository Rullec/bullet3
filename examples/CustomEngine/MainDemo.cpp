#include "MainDemo.h"

#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_X 1
#define ARRAY_SIZE_Z 1

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletGenDynamics/btGenController/btGenContactAwareAdviser.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <fstream>
#include <iostream>
#include <memory>

extern int global_frame_id;
bool gEnablePauseWhenSolveError, gEnableResolveWhenSolveError;
// static std::string gContactAwareTraj =
//     "/home/xudong/Projects/DeepMimic/data/id_test/sample_legs/"
//     "traj_legs_986044642.json";

static std::string gContactAwareTraj = "invalid_path.traj";
// std::string gOutputLogPath;
struct CustomEngineMainDemo : public CommonRigidBodyBase
{
    struct tParams
    {
        std::string mSimulatorConfigPath;
        bool mAddObj;
        bool mAddMultibody;
        bool mEnableGround;
        double mGroundHeight;
        std::string mMultibodyPath;
        int mObjLinkNum;
        std::string mLCPSolverConfigPath;
        std::string mObjType;
        bool mEnableObjPerturb;
        double mDefaultTimestep;
        int mPauseFrame;
        bool mEnableContactAwareControl;
        tParams(const std::string &path);
    };

    struct tParams *physics_param;
    CustomEngineMainDemo(struct GUIHelperInterface *helper);
    virtual ~CustomEngineMainDemo();

    virtual void stepSimulation(float deltaTime) override final;
    virtual void initPhysics() override;
    virtual void exitPhysics() override;
    virtual void renderScene();
    void resetCamera()
    {
        float dist = 3;
        float pitch = -35;
        float yaw = 52;
        float targetPos[3] = {0, 0.46, 0};
        m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1],
                                 targetPos[2]);
    }

protected:
    btGeneralizeWorld *mGenWorld;
    btGenContactAwareAdviser *mAdviser;
    double mTimestep;
    // btRigidBody* target_rigidbody;
    // float mTime;
};
extern bool gPauseSimulation;
#include "valgrind/callgrind.h"
void CustomEngineMainDemo::stepSimulation(float dt)
{
    if (global_frame_id == 0)
        m_guiHelper->resetCamera(1.64, -267.6, 13.4, 0.0078, 0.4760, 0.4799);
    dt = physics_param->mDefaultTimestep;
    CALLGRIND_START_INSTRUMENTATION;
    if (physics_param->mEnableContactAwareControl && mAdviser->IsEnd())
    {
        std::cout << "traj terminated without save\n";
        exit(0);
        mAdviser->Reset();
        mAdviser->SetTraj(gContactAwareTraj, "tmp_traj.json", true);
    }
    mGenWorld->ClearForce();
    mGenWorld->StepSimulation(
        static_cast<float>(physics_param->mDefaultTimestep));
    if (physics_param->mPauseFrame == global_frame_id)
    {
        gPauseSimulation = true;
    }
    global_frame_id++;
    CALLGRIND_STOP_INSTRUMENTATION;
    // CommonRigidBodyBase::stepSimulation(dt);
    // if (mTime > 1)
    // {
    // 	m_dynamicsWorld->removeCollisionObject(target_rigidbody);
    // 	m_collisionShapes.pop_back();
    // }
    // std::cout <<"collision shapes = " << m_collisionShapes.size() <<
    // std::endl;
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void CustomEngineMainDemo::exitPhysics() { CALLGRIND_DUMP_STATS; }
void CustomEngineMainDemo::initPhysics()
{
    srand(0);
    physics_param = new tParams("./examples/CustomEngine/config.json");

    m_guiHelper->setUpAxis(1);

    // cSimulator::tParams simulator_params;
    // if (physics_param->mEnableGravity)
    // 	simulator_params.mGravity = tVector(0, -9.8, 0, 0);
    // else
    // 	simulator_params.mGravity = tVector(0, 0, 0, 0);

    // simulator_params.mb_damping1 = physics_param->mMBDamping1;
    // simulator_params.mb_damping2 = physics_param->mMBDamping2;
    // simulator_params.mb_scale = physics_param->mMBScale;
    // simulator_params.mb_enable_rk4 = physics_param->mEnableRK4MB;
    // simulator_params.rigid_damping = physics_param->mRigidDamping;

    // if (physics_param->mEnableLCP == true)
    // {
    // 	simulator_params.Mode = cSimulator::eContactResponseMode::LCPMode;
    // 	simulator_params.mLCPConfigPath = physics_param->mLCPSolverConfigPath;
    // }

    // else
    // 	simulator_params.Mode = cSimulator::eContactResponseMode::PenaltyMode;

    mGenWorld = new btGeneralizeWorld();
    mGenWorld->Init(physics_param->mSimulatorConfigPath);
    /*
        "reference_traj_long":
"/home/xudong/Projects/DeepMimic/data/batch_trajs/0827/traj_fullbody_799585164_1500_to_3000.json",
"reference_traj":
"/home/xudong/Projects/DeepMimic/data/batch_trajs/0827/traj_fullbody_799585164.json",
"reference_traj_walk":
"/home/xudong/Projects/DeepMimic/logs/tmp/test_traj_full.json",
"reference_traj_bak1":
"/home/xudong/Projects/DeepMimic/logs/tmp/test_traj_legs.json",
"reference_traj_legs":
"/home/xudong/Projects/DeepMimic/logs/tmp/test_traj_3links.json",
"reference_traj_stand":
"/home/xudong/Projects/DeepMimic/logs/tmp/test_traj_2links.json",
    */

    m_dynamicsWorld = mGenWorld->GetInternalWorld();

    if (physics_param->mAddObj)
        mGenWorld->AddObj(physics_param->mObjLinkNum, physics_param->mObjType,
                          physics_param->mEnableObjPerturb);
    if (physics_param->mAddMultibody)
    {
        mGenWorld->AddMultibody(physics_param->mMultibodyPath);
        if (physics_param->mEnableContactAwareControl)
        {
            mGenWorld->SetEnableContacrAwareControl();
            mAdviser = mGenWorld->GetContactAwareAdviser();
            mAdviser->SetTraj(gContactAwareTraj, "tmp_traj.json", true);
        }
    }

    if (physics_param->mEnableGround)
        mGenWorld->AddGround(physics_param->mGroundHeight);

    // {
    // 	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(1.),
    // btScalar(1.), btScalar(1.))); 	btVector3 localInertia(0, 0, 0);
    // 	groundShape->calculateLocalInertia(0, localInertia);
    // 	btRigidBody* body = new btRigidBody(0, 0, groundShape, localInertia);

    // 	body->setUserIndex(-1);
    // 	m_dynamicsWorld->addRigidBody(body);
    // }

    // {
    // 	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(0.1),
    // btScalar(0.1), btScalar(0.1))); 	btVector3 localInertia(0, 0, 0);
    // 	groundShape->calculateLocalInertia(0, localInertia);
    // 	btCollisionObject * obj = new btCollisionObject();
    // 	obj->setCollisionShape(groundShape);
    // 	// btRigidBody* body = new btRigidBody(0, 0, groundShape, localInertia);

    // 	obj->setUserIndex(-1);
    // 	m_dynamicsWorld->addCollisionObject(obj);
    // }
    /*
    camera set distance 1.6400
    camera set pitch 13.4000
    camera set yaw -267.6000
    camera set pos 0.0078 0.4760 0.4799
    */

    // m_guiHelper->resetCamera(1.64, -267.6, 13.4, 0.0078, 0.4760, 0.4799);
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
    // m_guiHelper->resetCamera(1.64, -267.6, 13.4, 0.0078, 0.4760, 0.4799);
}

void CustomEngineMainDemo::renderScene() { CommonRigidBodyBase::renderScene(); }

CustomEngineMainDemo::tParams::tParams(const std::string &path)
{
    Json::Value json_root;
    btJsonUtil::LoadJson(path, json_root);
    mSimulatorConfigPath =
        btJsonUtil::ParseAsString("simulator_path", json_root);
    mAddMultibody = json_root["add_multibody"].asBool();
    mMultibodyPath = json_root["multibody_path"].asString();
    mAddObj = json_root["add_obj"].asBool();
    mEnableGround = json_root["enable_ground"].asBool();
    mGroundHeight = btJsonUtil::ParseAsDouble("ground_height", json_root);
    mObjLinkNum = json_root["obj_num"].asInt();
    mObjType = json_root["obj_type"].asString();
    mEnableObjPerturb = json_root["enable_obj_perturb"].asBool();
    mDefaultTimestep = json_root["default_timestep"].asDouble();
    mPauseFrame = json_root["pause_frame"].asInt();
    mEnableContactAwareControl =
        btJsonUtil::ParseAsBool("enable_contact_aware_control", json_root);
    gEnablePauseWhenSolveError =
        json_root["enable_pause_when_solved_wrong"].asBool();
    gEnableResolveWhenSolveError =
        json_root["enable_resolve_when_solved_wrong"].asBool();
    gContactAwareTraj = json_root["contact_aware_ref_traj"].asString();
    // gOutputLogPath = json_root["output_log_path"].asString();
}

CommonExampleInterface *CustomMainCreateFunc(CommonExampleOptions &options)
{
    return new CustomEngineMainDemo(options.m_guiHelper);
}

CustomEngineMainDemo::~CustomEngineMainDemo()
{
    if (physics_param)
        delete physics_param;
    if (mGenWorld)
        delete mGenWorld;
}

CustomEngineMainDemo::CustomEngineMainDemo(struct GUIHelperInterface *helper)
    : CommonRigidBodyBase(helper)
{
    // mTime = 0;
    physics_param = nullptr;
    mGenWorld = nullptr;
}