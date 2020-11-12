#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconController.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSM.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"

btGenSimbiconController::btGenSimbiconController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::SimbiconController, world)
{
    mFSM = nullptr;
    mPDController = nullptr;
}

btGenSimbiconController::~btGenSimbiconController()
{
    delete mFSM;
    mFSM = nullptr;
    delete mPDController;
    mPDController = nullptr;
}

/**
 * \brief               Initialize the simbicon controller
*/
void btGenSimbiconController::Init(cRobotModelDynamics *model,
                                   const std::string &conf)
{
    btGenControllerBase::Init(model, conf);
    // 1. check the skeleton(fixed now)
    std::string char_file = model->GetCharFile();
    BTGEN_ASSERT(char_file ==
                 "../DeepMimic/data/0908/characters/skeleton_legs.json");

    Json::Value root;
    btJsonUtil::LoadJson(conf, root);
    const Json::Value &fsm_config = btJsonUtil::ParseAsValue("FSM", root);
    std::string pd_ctrl_path =
        btJsonUtil::ParseAsString("pd_controller_path", root);
    const Json::Value &balance_ctrl =
        btJsonUtil::ParseAsValue("balance_control", root);

    // 2. build FSM
    BuildFSM(fsm_config);
    // 3. build & correct PD controller
    BuildPDCtrl(pd_ctrl_path);
    // 4. build the balance policy
    BuildBalanceCtrl(balance_ctrl);
}

/**
 * \brief               Update simbicon controller, calculate & apply the control force
 * 1. input current time, input contact points(contact manager), update FSM, output target pose
 * 2. input target pose into balance feedback module, output target pose
 * 3. input the target pose into PD controller, output control force
 *       3.1 for stance hip, use the inverse torque and don't track the ref pose
 *       3.2 for torso and swing hip, the torque is calculated in world frame
 *       3.3 for other joints, just track the local angle
*/
void btGenSimbiconController::Update(double dt)
{
    btGenControllerBase::Update(dt);
}

void btGenSimbiconController::Reset() {}

/**
 * \brief           Build FSM
*/
void btGenSimbiconController::BuildFSM(const Json::Value &conf)
{
    mFSM = new btGenFSM(mWorld, mModel, conf);
}
/**
 * \brief           Build the PD controller
*/
void btGenSimbiconController::BuildPDCtrl(const std::string &pd_path)
{
    mPDController = new btGenPDController(mWorld);
    mPDController->Init(mModel, pd_path);
}
/**
 * \brief           Build the balance control policy
*/
void btGenSimbiconController::BuildBalanceCtrl(const Json::Value &conf)
{
    printf("[warn] balance control absent\n");
}