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
#include "BulletGenDynamics/btGenController/ContactAwareController/FBFCalculator/btGenFrameByFrameCalculator.h"
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <fstream>
#include <iostream>
#include <memory>

extern int global_frame_id;
bool gEnablePauseWhenSolveError, gEnableResolveWhenSolveError;

static float init_camDist, init_yaw, init_pitch, init_camPosX, init_camPosY,
    init_camPosZ;

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
        bool mEnableController;
        std::string mControllerConfigPath;
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
        m_guiHelper->resetCamera(init_camDist, init_yaw, init_pitch,
                                 init_camPosX, init_camPosY, init_camPosZ);
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
    {
        // set init camera position
        resetCamera();
    }

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

        // add controller
        if (physics_param->mEnableController)
        {
            mGenWorld->AddController(physics_param->mControllerConfigPath);
            mGenWorld->GetController()->SetBulletGUIHelperInterface(
                m_guiHelper);

            // if it is the contact aware controller, has some speical settings
            if (physics_param->mEnableContactAwareControl)
            {
                if (mGenWorld->HasContactAwareController() == false)
                {
                    printf(
                        "[error] illegal contact aware controller setting\n");
                    exit(0);
                }
                mAwareController = mGenWorld->GetContactAwareController();
                mAwareController->SetTraj(gContactAwareTraj, "tmp_traj.json",
                                          true);
            }
        }
        // Test();
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
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
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
    gEnablePauseWhenSolveError =
        json_root["enable_pause_when_solved_wrong"].asBool();
    gEnableResolveWhenSolveError =
        json_root["enable_resolve_when_solved_wrong"].asBool();
    gContactAwareTraj = json_root["contact_aware_ref_traj"].asString();
    mRealTimeSim = btJsonUtil::ParseAsBool("real_time_sim", json_root);
    mEnableController = btJsonUtil::ParseAsBool("enable_controller", json_root);

    mControllerConfigPath = btJsonUtil::ParseAsString("ctrl_config", json_root);
    mEnableContactAwareControl =
        btJsonUtil::ParseAsBool("enable_contact_aware_control", json_root);
    if (mEnableContactAwareControl == true && mEnableController == false)
    {
        printf("[error] Maindemo: contact aware is enabled but controller is "
               "disabled, illegal\n");
        exit(0);
    }

    tVectorXd camera_params;
    btJsonUtil::ReadVectorJson(
        btJsonUtil::ParseAsValue("camera_init_param", json_root),
        camera_params);
    init_camDist = camera_params[0];
    init_yaw = camera_params[1];
    init_pitch = camera_params[2];
    init_camPosX = camera_params[3];
    init_camPosY = camera_params[4];
    init_camPosZ = camera_params[5];
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
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
void CustomEngineMainDemo::Test()
{
    std::cout << "begin test\n";
    auto mb = mGenWorld->GetMultibody();
    mb->SetComputeThirdDerive(true);
    int dof = mb->GetNumOfFreedom();
    tVectorXd q = tVectorXd::Random(dof), qdot = tVectorXd::Random(dof);
    mb->SetqAndqdot(q, qdot);

    // for (int i = 0; i < mb->GetNumOfLinks(); i++)
    // {
    //     auto joint = dynamic_cast<Joint *>(mb->GetJointById(i));
    //     std::cout << "joint " << i << " type = \n"
    //               << joint->GetJointType() << std::endl;
    // }
    // exit(0);
    mb->TestmTqq();
    mb->TestmTqqq();
    mb->TestmWq();
    mb->TestmWqq();
    mb->TestmWqqq();

    mb->TestdMassMatrixdq();
    mb->TestSecondJacobian();
    mb->TestThirdJacobian();
    mb->TestdJdotdq();
    mb->TestJacobian();

    mb->TestdJdotdq();
    mb->TestSecondJacobian();
    mb->TestReducedAPI();
    mb->TestThirdJacobian();
    mb->TestdJdotdqdot();
    mb->TestDCoriolisMatrixDq();
    mb->TestDCoriolisMatrixDqdot();
    mb->TestSetFreedomValueAndDot();
    mb->TestConvertGenForceToJointTorque();
    std::cout << "test done\n";
    exit(0);
}