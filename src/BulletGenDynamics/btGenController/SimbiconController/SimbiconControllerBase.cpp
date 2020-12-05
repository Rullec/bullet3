#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconControllerBase.h"
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconState.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"

btGenSimbiconControllerBase::btGenSimbiconControllerBase(
    btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::SimbiconController, world)
{

    mPDController = new btGenPDController(mWorld);
    mRefTrajModel = nullptr;
    mStates.clear();
    mStateIdx = -1;
    mStance = LEFT_STANCE;
    mRootId = -1;
    mStanceFoot = mSwingFoot = nullptr;
    mSwingHipIdx = mStanceHipIdx = -1;
    mLeftFoot = mRightFoot = nullptr;
    mLeftHipIdx = mRightHipIdx = -1;
    mInitPose.resize(0);
    mD.setZero();
    mV.setZero();
    mPhi = 0;
}

/**
 * \brief           dtor
*/
btGenSimbiconControllerBase::~btGenSimbiconControllerBase()
{
    for (auto &x : mStates)
        delete x;
    mStates.clear();
    delete mPDController;
    delete mRefTrajModel;
}

/**
 * \brief           Initialize the Simbicon controller
 * 
 *      1. create PD controller
 *      2. init states
 *      3. character's init pose
 *      4. set start state
 *      5. set start stance
*/
void btGenSimbiconControllerBase::Init(cRobotModelDynamics *model,
                                       const std::string &conf_str)
{
    btGenControllerBase::Init(model, conf_str);
    Json::Value root;
    btJsonUtil::LoadJson(conf_str, root);

    // 1. create PD controller
    {
        std::string pd_path =
            btJsonUtil::ParseAsString("pd_controller_path", root);
        mPDController->Init(model, pd_path);
    }

    // init control joint info (feet, hips)
    InitJointInfo();

    // 2. create States
    InitStates(root);

    // 3. init starting pose
    InitStartPose(root);

    // 4. create start state
    mStateIdx = btJsonUtil::ParseAsInt("init_state", root);
    BTGEN_ASSERT(mStateIdx < mStates.size());
    TransiteToState(mStateIdx);

    // 5. create start stance
    std::string init_stance_str =
        btJsonUtil::ParseAsString("init_stance", root);
    if (init_stance_str == LEFT_STANCE_STR)
        mStance = LEFT_STANCE;
    else if (init_stance_str == RIGHT_STANCE_STR)
        mStance = RIGHT_STANCE;
    else
    {
        BTGEN_ASSERT(false);
    }
    SetStance(mStance);
}

/**
 * \brief           Update the simbicon controller
 * 1. calculate the control torques
 * 2. apply the control torques
 * 3. update the states
*/
void btGenSimbiconControllerBase::Update(double dt)
{
    // 1. caluclate the contorl torques
    tEigenArr<btGenPDForce> forces(0);
    ComputeTorques(forces);

    // 2. apply the control torques
    ApplyTorques(forces);
}
void btGenSimbiconControllerBase::Reset() {}
void btGenSimbiconControllerBase::UpdateDandV() {}
void btGenSimbiconControllerBase::GetTargetPose() {}
void btGenSimbiconControllerBase::ComputeTorques(
    tEigenArr<btGenPDForce> &forces)
{
}
double btGenSimbiconControllerBase::GetStanceFootWeightRatio()
{
    BTGEN_ASSERT(false);
    return -1;
}
void btGenSimbiconControllerBase::ComputeHipTorques() {}
void btGenSimbiconControllerBase::AdvanceInTime() {}

/**
 * \brief           Create simbicon states
 *      Given the json value, parse all states, create the trajectories and the base trajectories for each component
*/
void btGenSimbiconControllerBase::InitStates(const Json::Value &conf)
{
    int num_of_states = btJsonUtil ::ParseAsInt("num_of_states", conf);
    std::string STATE_BASE_STR = "state_";
    mStates.clear();
    for (int i = 0; i < num_of_states; i++)
    {
        std::string state_str = STATE_BASE_STR + std::to_string(i);
        const Json::Value &state_json =
            btJsonUtil::ParseAsValue(state_str, conf);
        auto state = new btGenSimbiconState(state_json, i);
        mStates.push_back(state);
    }
}

/**
 * \brief           Create init pose
*/
void btGenSimbiconControllerBase::InitStartPose(const Json::Value &conf)
{
    std::string st_pose = btJsonUtil::ParseAsString("init_pose", conf);
    Json::Value pose;
    btJsonUtil::LoadJson(st_pose, pose);
    int dof = mModel->GetNumOfFreedom();
    BTGEN_ASSERT(pose.size() == dof);
    tVectorXd q = btJsonUtil::ReadVectorJson(pose);
    mModel->SetqAndqdot(q, tVectorXd::Zero(dof));
}

/**
 * \brief               Init joint info
 *      given character model, set foot ptrs and indices
*/
void btGenSimbiconControllerBase::InitJointInfo()
{
    std::string LEFT_FOOT_STR = "LeftFootTongue_joint",
                RIGHT_FOOT_STR = "RightFootTongue_joint",
                LEFT_HIP_STR = "LeftLeg_joint",
                RIGHT_HIP_STR = "RightLeg_joint";

    mLeftFoot = dynamic_cast<Joint *>(mModel->GetJoint(LEFT_FOOT_STR));
    mRightFoot = dynamic_cast<Joint *>(mModel->GetJoint(RIGHT_FOOT_STR));
    BTGEN_ASSERT(mLeftFoot != nullptr);
    BTGEN_ASSERT(mRightFoot != nullptr);
    auto left_hip_ptr = mModel->GetJoint(LEFT_HIP_STR),
         right_hip_ptr = mModel->GetJoint(RIGHT_HIP_STR);
    BTGEN_ASSERT(left_hip_ptr != nullptr);
    BTGEN_ASSERT(right_hip_ptr != nullptr);
    mLeftHipIdx = left_hip_ptr->GetId();
    mRightHipIdx = right_hip_ptr->GetId();

    printf("[debug] left foot name = %s\n", mLeftFoot->GetName().c_str());
    printf("[debug] right foot name = %s\n", mRightFoot->GetName().c_str());
    printf("[debug] left hip id = %d\n", mLeftHipIdx);
    printf("[debug] right hip id = %d\n", mRightHipIdx);
}

/**
 * \brief               Apply the control torques
*/
void btGenSimbiconControllerBase::ApplyTorques(
    const tEigenArr<btGenPDForce> &forces)
{
}

/**
 * \brief               Transite to a new state
 * \param state         state index
*/
void btGenSimbiconControllerBase::TransiteToState(int state)
{
    SetFSMStateTo(state);
}

/**
 * \brief           set the fsm state to antoher value
*/
void btGenSimbiconControllerBase::SetFSMStateTo(int state_idx)
{
    BTGEN_ASSERT(state_idx >= 0 && state_idx < mStates.size());
    mStateIdx = state_idx;
    SetStance(mStates[mStateIdx]->GetStateStance(mStance));
}

/**
 * \brief           set the new stance configuration
*/
void btGenSimbiconControllerBase::SetStance(int new_stance)
{
    this->mStance = new_stance;

    if (mStance == LEFT_STANCE)
    {
        mStanceFoot = mLeftFoot;
        mSwingFoot = mRightFoot;
        mStanceHipIdx = mLeftHipIdx;
        mSwingHipIdx = mRightHipIdx;
    }
    else if (mStance == RIGHT_STANCE)
    {
        mStanceFoot = mRightFoot;
        mSwingFoot = mLeftFoot;
        mStanceHipIdx = mRightHipIdx;
        mSwingHipIdx = mLeftHipIdx;
    }
    else
    {
        BTGEN_ASSERT(false);
    }
}