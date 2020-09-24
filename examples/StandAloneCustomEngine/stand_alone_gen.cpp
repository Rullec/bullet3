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
#include "BulletGenDynamics/btGenController/btGenContactAwareAdviser.h"
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
btGenContactAwareAdviser *mAdviser;
btDiscreteDynamicsWorld *m_dynamicsWorld;
double mTimestep;

void stepSimulation(float dt);
void initPhysics();
int global_frame_id = 0;

int main(int argc, char *argv[])
{
    srand(0);
    initPhysics();
    mTimestep = 1.0 / 600;
    while (true)
    {
        stepSimulation(mTimestep);
    }
    return 0;
}
void stepSimulation(float dt)
{
    if (mAdviser->IsEnd())
    {
        std::cout << "traj terminated without save\n";
        mAdviser->Reset();
        mAdviser->SetTraj(gContactAwareTraj, "tmp_traj.json", true);
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
    physics_param = new tParams("./examples/CustomEngine/config.json");

    mGenWorld = new btGeneralizeWorld();
    mGenWorld->Init(physics_param->mSimulatorConfigPath);

    m_dynamicsWorld = mGenWorld->GetInternalWorld();

    if (physics_param->mAddObj)
        mGenWorld->AddObj(physics_param->mObjLinkNum, physics_param->mObjType,
                          physics_param->mEnableObjPerturb);
    if (physics_param->mAddMultibody)
    {
        mGenWorld->AddMultibody(physics_param->mMultibodyPath);
        mGenWorld->SetEnableContacrAwareControl();
        mAdviser = mGenWorld->GetContactAwareAdviser();
        mAdviser->SetTraj(gContactAwareTraj, "tmp_traj.json", true);
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