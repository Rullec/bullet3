#include "BulletGenDynamics/btGenWorld.h"
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
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <fstream>
#include <iostream>
#include <memory>
static std::string gContactAwareTraj = "invalid_path.traj";
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
    tParams(const std::string &path);
};
struct tParams *physics_param;
btGeneralizeWorld *mGenWorld;
btGenContactAwareController *mController;
btDiscreteDynamicsWorld *m_dynamicsWorld;
double mTimestep;
bool gPauseSimulation;
void stepSimulation(float dt);
void initPhysics();
void setFBFCalculatorCoefAndRefTraj(int argc, char *argv[]);
// void setControllerRefTraj(int argc, char *argv[]);
int global_frame_id = 0;

int main(int argc, char *argv[])
{
    srand(0);
    initPhysics();
    setFBFCalculatorCoefAndRefTraj(argc, argv);
    mTimestep = 1.0 / 600;
    while (true)
    {
        stepSimulation(mTimestep);
    }
    return 0;
}

void stepSimulation(float dt)
{
    if (mController->IsEnd())
    {
        std::cout << "traj terminated without save\n";
        mController->Reset();
        mController->SetTraj(gContactAwareTraj, "tmp_traj.json", true);
    }
    if (mGenWorld->GetMultibody()->IsCartesianMaxVel() == true)
    {
        std::cout
            << "[error] the cartesian velocity of model has exploded, exit\n";
        exit(0);
    }
    mGenWorld->ClearForce();
    mGenWorld->StepSimulation(
        static_cast<float>(physics_param->mDefaultTimestep));
    global_frame_id++;
    // CommonRigidBodyBase::stepSimulation(dt);
    // if (mTime > 1)
    // {
    // 	m_dynamicsWorld->removeCollisionObject(target_rigidbody);
    // 	m_collisionShapes.pop_back();
    // }
    // std::cout <<"collision shapes = " << m_collisionShapes.size() <<
    // std::endl;
}

void initPhysics()
{
    physics_param =
        new tParams("./examples/CustomEngine/sim_configs/config.json");

    mGenWorld = new btGeneralizeWorld();
    mGenWorld->Init(physics_param->mSimulatorConfigPath);

    m_dynamicsWorld = mGenWorld->GetInternalWorld();

    if (physics_param->mAddObj)
        mGenWorld->AddObj(physics_param->mObjLinkNum, physics_param->mObjType,
                          physics_param->mEnableObjPerturb);
    if (physics_param->mAddMultibody)
    {
        mGenWorld->AddMultibody(physics_param->mMultibodyPath);
        // mGenWorld->AddController();
        // mController = mGenWorld->GetContactAwareController();
        // mController->SetTraj(gContactAwareTraj, "tmp_traj.json", true);
    }

    if (physics_param->mEnableGround)
        mGenWorld->AddGround(physics_param->mGroundHeight);
}

tParams::tParams(const std::string &path)
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
    gContactAwareTraj = json_root["contact_aware_ref_traj"].asString();
    // gOutputLogPath = json_root["output_log_path"].asString();
}

void setFBFCalculatorCoefAndRefTraj(int argc, char *argv[])
{
    std::cout << "main begin set fbf calculator coeff, argc = " << argc
              << std::endl;
    Json::Value root;
    std::string ref_traj = "";
    for (int i = 1; i < argc; i += 2)
    {
        // for odd i, it should be label
        // for even i, it should be value
        std::string label = std::string(argv[i]);
        if (label.substr(0, 4) == "coef")
        {
            // coef label
            std::string name = label.substr(5);
            if (i + 1 >= argc)
            {
                std::cout << "[error] no value for label " << label
                          << std::endl;
                exit(0);
            }
            double value = std::atof(argv[i + 1]);
            std::cout << "coef name = " << name << " value = " << value
                      << std::endl;
            if (root.isMember(name) == true)
            {
                std::cout << "[error] key " << name
                          << " appears for two times, exit\n";
            }

            root[name] = value;
        }
        else if (label.substr(0, 4) == "traj")
        {
            if (ref_traj.size() != 0)
            {
                std::cout << "[error] ref traj size != 0, it is " << ref_traj
                          << std::endl;
                exit(0);
            }
            ref_traj = std::string(argv[i + 1]);
        }
        else
        {
            std::cout << "[error] token " << label
                      << " cannot be recognized, exit\n";
            exit(0);
        }
    }
    if (ref_traj.size() == 0)
    {
        std::cout << "[error] ref traj doesn't get in argv\n";
        exit(1);
    }
    std::cout << "[log] set ref traj " << ref_traj << std::endl;
    std::cout << "[log] set coef " << root << std::endl;

    mController->GetTargetCalculator()->SetCoef(root);
    mController->SetTraj(ref_traj, "tmp.traj", true);
}

void setControllerRefTraj(int argc, char *argv[])
{
    std::cout << "[main] begin to set controller ref traj\n";
    exit(0);
}