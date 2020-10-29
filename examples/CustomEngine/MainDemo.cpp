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
#include "BulletGenDynamics/btGenController/FBFCalculator/btGenFrameByFrameCalculator.h"
#include "BulletGenDynamics/btGenController/btGenContactAwareController.h"
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
        bool mRealTimeSim;
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
    void MultistepSim(float dt);
    void SinglestepSim(float dt);
    void Test();
    btGeneralizeWorld *mGenWorld;
    btGenContactAwareController *mAwareController;
    double mTimestep;
    // btRigidBody* target_rigidbody;
    // float mTime;
};
extern bool gPauseSimulation;
// #include "valgrind/callgrind.h"
void CustomEngineMainDemo::stepSimulation(float dt)
{
    if (global_frame_id == 0)
        m_guiHelper->resetCamera(1.64, -267.6, 13.4, 0.0078, 0.4760, 0.4799);

    if (physics_param->mRealTimeSim == true)
    {
        MultistepSim(dt);
    }
    else
    {
        dt = physics_param->mDefaultTimestep;
        SinglestepSim(dt);
    }
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void CustomEngineMainDemo::exitPhysics()
{
    // CALLGRIND_DUMP_STATS;
}
void CustomEngineMainDemo::initPhysics()
{
    srand(0);
    physics_param =
        new tParams("./examples/CustomEngine/sim_configs/config.json");

    m_guiHelper->setUpAxis(1);

    mGenWorld = new btGeneralizeWorld();
    mGenWorld->Init(physics_param->mSimulatorConfigPath);

    m_dynamicsWorld = mGenWorld->GetInternalWorld();

    if (physics_param->mAddObj)
        mGenWorld->AddObj(physics_param->mObjLinkNum, physics_param->mObjType,
                          physics_param->mEnableObjPerturb);
    if (physics_param->mAddMultibody)
    {
        mGenWorld->AddMultibody(physics_param->mMultibodyPath);
        Test();
        if (physics_param->mEnableContactAwareControl)
        {
            mGenWorld->SetEnableContacrAwareControl();
            mAwareController = mGenWorld->GetContactAwareController();
            mAwareController->SetTraj(gContactAwareTraj, "tmp_traj.json", true);
            mAwareController->SetBulletGUIHelperInterface(m_guiHelper);
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
    mRealTimeSim = btJsonUtil::ParseAsBool("real_time_sim", json_root);
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

void CustomEngineMainDemo::MultistepSim(float dt)
{
    int num_substep = 0;
    while (dt > 0)
    {
        if (physics_param->mPauseFrame == global_frame_id)
        {
            gPauseSimulation = true;
            global_frame_id++;
            break;
        }
        else
        {

            double elasped_time = physics_param->mDefaultTimestep;
            if (dt <= elasped_time)
                elasped_time = dt;
            dt -= elasped_time;

            if (physics_param->mEnableContactAwareControl &&
                mAwareController->IsEnd())
            {
                std::cout << "traj terminated without save\n";
                exit(0);
                mAwareController->Reset();
                mAwareController->SetTraj(gContactAwareTraj, "tmp_traj.json",
                                          true);
            }
            mGenWorld->ClearForce();
            mGenWorld->StepSimulation(
                static_cast<float>(physics_param->mDefaultTimestep));

            global_frame_id++;
        }
        num_substep++;
    }
    std::cout << "[log] num of substeps = " << num_substep << std::endl;
}
void CustomEngineMainDemo::SinglestepSim(float dt)
{
    if (std::fabs(dt - physics_param->mDefaultTimestep) > 1e-10)
    {
        std::cout << "[error] single step sim " << dt << " != default dt "
                  << physics_param->mDefaultTimestep << std::endl;
        exit(0);
    }
    if (physics_param->mPauseFrame == global_frame_id)
    {
        gPauseSimulation = true;
    }
    else
    {
        mGenWorld->ClearForce();
        mGenWorld->StepSimulation(
            static_cast<float>(physics_param->mDefaultTimestep));
    }

    global_frame_id++;
    return;
}
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
void CustomEngineMainDemo::Test()
{
    std::cout << "begin test\n";
    auto mb = mGenWorld->GetMultibody();
    mb->SetComputeThirdDerive(true);
    int dof = mb->GetNumOfFreedom();
    // tVectorXd q = tVectorXd::Random(dof), qdot = tVectorXd::Random(dof);
    tVectorXd q = tVectorXd::Zero(dof), qdot = tVectorXd::Zero(dof);
    mb->SetqAndqdot(q, qdot);

    // mb->TestmWq();
    // mb->TestmTqq();
    // mb->TestmWqq();
    // mb->TestmTqqq();
    // mb->TestmWqqq();
    // mb->TestSecondJacobian();
    mb->TestThirdJacobian();
    // mb->TestJacobian();
    // mb->TestLinkdMassMatrixdq();
    // mb->TestdGenGravitydq(tVector::Random());
    // mb->TestdMassMatrixdq();
    std::cout << "test done\n";
    exit(0);
}