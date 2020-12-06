#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconControllerBase.h"
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconState.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ContactSolver.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
// #define APPLY_ROOT_TORQUE

btGenSimbiconControllerBase::btGenSimbiconControllerBase(
    btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::SimbiconController, world)
{

    mPDController = new btGenPDController(mWorld);
    mRefTrajModel = nullptr;
    mStates.clear();
    mStateIndex = -1;
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
    mStateIndex = btJsonUtil::ParseAsInt("init_state", root);
    BTGEN_ASSERT(mStateIndex < mStates.size());
    TransiteToState(mStateIndex);

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

    // init ref model
    mRefTrajModel = new cRobotModelDynamics();

    mRefTrajModel->Init(mModel->GetCharFile().c_str(), mModel->GetScale(),
                        ModelType::JSON);
    mRefTrajModel->InitSimVars(mWorld, true, true, false);
    UpdateRefModel(tVectorXd::Zero(mModel->GetNumOfFreedom()));
}

/**
 * \brief           Update the simbicon controller
 * 1. calculate the control torques
 * 2. apply the control torques
 * 3. update the states
*/
void btGenSimbiconControllerBase::Update(double dt)
{
    ClearLines();
    // 1. caluclate the contorl torques
    tEigenArr<btGenPDForce> forces(0);
    ComputeTorques(dt, forces);

    // 2. apply the control torques
    ApplyTorques(forces);

    // 3. update controller
    AdvanceInTime(dt);

    // 4. draw contact points
    {
        // auto mana = mWorld->GetContactManager();
        // for (auto &f : mWorld->GetContactForces())
        // {
        //     if (f->mPassiveObj == mWorld->GetGround())
        //     {
        //         tVector3d st = f->mWorldPos.segment(0, 3);
        //         tVector3d ed = st + f->mForce.segment(0, 3) / 100;
        //         DrawLine(st, ed);
        //     }
        // }
    }
}
void btGenSimbiconControllerBase::Reset() { BTGEN_ASSERT(false); }

/**
 * \brief           calculate the vector d and v in character frame
*/
void btGenSimbiconControllerBase::UpdateDandV()
{
    // 1. d = com - stance_foot
    tVector3d stance_foot =
                  mModel->GetLinkById(mStanceFoot->GetId())->GetWorldPos(),
              com_pos = this->mModel->GetCoMPosition();
    mD = com_pos - stance_foot;
    std::cout << "[debug] world D = " << mD.transpose() << std::endl;
    // 2. v = com_vel
    mV = mModel->GetComVelocity();
    std::cout << "[debug] world V = " << mV.transpose() << std::endl;

    // 3. get charcter frame, convert the vector from world frame to char frame
    double heading = mModel->GetHeading();
    std::cout << "[debug] heading = " << heading << std::endl;
    tMatrix3d char_to_world =
        btMathUtil::AxisAngleToRotmat(
            tVector(0, 1, 0, 0) *
            heading) // rotation a vector in char frame to world frame
            .block(0, 0, 3, 3);
    mD = char_to_world.transpose() * mD;
    mV = char_to_world.transpose() * mV;
    std::cout << "[debug] char D = " << mD.transpose() << std::endl;
    std::cout << "[debug] char V = " << mV.transpose() << std::endl;
}
void btGenSimbiconControllerBase::GetTargetPose() {}

/**
 * \brief           compute the simbicon control torques on each joint
*/
void btGenSimbiconControllerBase::ComputeTorques(
    double dt, tEigenArr<btGenPDForce> &forces)
{
    forces.clear();

    // 1. update d and v in character frame
    UpdateDandV();

    // 2. compute the target pose
    int dof = mModel->GetNumOfFreedom();
    tVectorXd tar_pose = tVectorXd::Zero(dof), tar_vel = tVectorXd::Zero(dof);
    btGenSimbiconState *cur_state = mStates[mStateIndex];
    for (int i = 0; i < cur_state->GetTrajectoryCount(); i++)
    {
        auto cur_traj = cur_state->mTrajs[i];
        int joint_idx = cur_traj->getJointIndex(mStance);
        auto cur_joint = dynamic_cast<Joint *>(mModel->GetJointById(joint_idx));

        // get the local target of joint
        tQuaternion local_target = cur_traj->evaluateTrajectory(
            this, cur_joint, mStance, mPhi, mD, mV);
        int offset = cur_joint->GetOffset(),
            size = cur_joint->GetNumOfFreedom();
        // change the target in the bank
        switch (cur_joint->GetJointType())
        {
        case JointType::SPHERICAL_JOINT:
        {
            // local_target to euler angles
            BTGEN_ASSERT(size == 3);
            tar_pose.segment(offset, size) =
                btMathUtil::QuaternionToEulerAngles(local_target,
                                                    btRotationOrder::bt_XYZ)
                    .segment(0, 3);
            break;
        }
        case JointType::REVOLUTE_JOINT:
        {
            BTGEN_ASSERT(size == 1);
            tar_pose[offset] = btMathUtil::QuaternionToEulerAngles(
                local_target, btRotationOrder::bt_XYZ)[0];
            break;
        }
        default:
            BTGEN_ASSERT(false);
            break;
        }
    }
    // 3. compute the explicit PD force on each joint
    mPDController->SetPDTargetq(tar_pose);
    mPDController->SetPDTargetqdot(tar_vel);

    // set the torso and swing hip to have global target
    for (auto &x : mPDController->GetJointPDCtrls())
    {
        x->SetUseWorldCoord(false);
        if (x->GetJoint()->GetId() == mRootId)
        {
            x->SetUseWorldCoord(true);
            std::cout << "[log] set root to use world coord\n";
        }
        if (x->GetJoint()->GetId() == mSwingHipIdx)
        {
            x->SetUseWorldCoord(true);
            std::cout << "[log] set swing hip " << x->GetJoint()->GetName()
                      << " to use world coord\n";
        }
    }
    mPDController->CalculateControlForces(dt, forces);
    BTGEN_ASSERT(forces.size() == mModel->GetNumOfJoint());

// 4. compute hip torques
#ifndef APPLY_ROOT_TORQUE
    ComputeHipTorques(forces, GetStanceFootWeightRatio());
#endif

    // 5. update ref model
    // std::cout << "[debug] tar pose = " << tar_pose.transpose() << std::endl;
    UpdateRefModel(tar_pose);
}

#include "BulletGenDynamics/btGenSolver/ContactManager.h"
double btGenSimbiconControllerBase::GetStanceFootWeightRatio()
{
    auto manager = mWorld->GetContactManager();

    double stance_foot_force = manager->GetVerticalTotalForceWithGround(
               mModel->GetLinkCollider(mStanceFoot->GetId())),
           swing_foot_force = manager->GetVerticalTotalForceWithGround(
               mModel->GetLinkCollider(mSwingFoot->GetId()));

    return stance_foot_force / (stance_foot_force + swing_foot_force + 1e-6);
}

/**
 * \brief               Compute control torques on hips (stance hip and swing hip)
 * \param forces        ref 
*/
void btGenSimbiconControllerBase::ComputeHipTorques(
    tEigenArr<btGenPDForce> &forces, double stancehip_to_swinghip_ratio)
{
    std::cout << "[log] stance foot ratio = " << stancehip_to_swinghip_ratio
              << std::endl;
    // the PD forces include root force
    BTGEN_ASSERT(forces.size() == mModel->GetNumOfJoint());

    tVector swinghip_force = forces[mSwingHipIdx].mForce,
            stancehip_force = forces[mStanceHipIdx].mForce;

    std::cout << "[log] origin swing hip force = " << swinghip_force.transpose()
              << std::endl;
    std::cout << "[log] origin stance hip force = "
              << stancehip_force.transpose() << std::endl;
    tVector root_makeup_torque = tVector::Zero();
    for (int i = 0; i < mModel->GetNumOfJoint(); i++)
    {
        auto joint = mModel->GetJointById(i);
        if (joint->GetParentId() == mModel->GetRoot()->GetId())
        {
            // std::cout << "[debug] joint " << joint->GetName()
            //           << "'s parent is root\n";
            root_makeup_torque -= forces[i].mForce;
        }
    }
    root_makeup_torque -= forces[mRootId].mForce;

    stancehip_force += root_makeup_torque * stancehip_to_swinghip_ratio;
    swinghip_force += root_makeup_torque * (1 - stancehip_to_swinghip_ratio);

    {
        int stance_hip_limit =
                dynamic_cast<Joint *>(mModel->GetJointById(mStanceHipIdx))
                    ->GetTorqueLim(),
            swing_hip_limit =
                dynamic_cast<Joint *>(mModel->GetJointById(mSwingHipIdx))
                    ->GetTorqueLim();
        if (stancehip_force.norm() > stance_hip_limit)
        {
            stancehip_force =
                stancehip_force / stancehip_force.norm() * stance_hip_limit;
        }
        if (swinghip_force.norm() > swing_hip_limit)
        {
            swinghip_force =
                swinghip_force / swinghip_force.norm() * swing_hip_limit;
        }
    }
    forces[mStanceHipIdx].mForce = stancehip_force;
    forces[mSwingHipIdx].mForce = swinghip_force;
    std::cout << "[log] final swing hip force = " << swinghip_force.transpose()
              << std::endl;
    std::cout << "[log] final stance hip force = "
              << stancehip_force.transpose() << std::endl;
    std::cout << "[debug] joint root force = "
              << forces[mRootId].mForce.transpose() << std::endl;

    {
        tVector root_force_affected = tVector::Zero();
        for (int i = 0; i < mModel->GetNumOfJoint(); i++)
        {
            auto joint = mModel->GetJointById(i);
            if (joint->GetParentId() == mModel->GetRoot()->GetId())
            {
                root_force_affected += -forces[i].mForce;
            }
        }
        // std::cout << "root affected force = " << root_force_affected.transpose()
        //           << std::endl;
    }
}

void btGenSimbiconControllerBase::AdvanceInTime(double dt)
{
    mPhi += dt / mStates[this->mStateIndex]->GetStateTime();
    std::cout << "[debug] cur phi = " << mPhi << " stance = "
              << ((mStance == LEFT_STANCE) ? ("left") : ("right")) << std::endl;
    auto manger = mWorld->GetContactManager();
    double swing_force = manger->GetVerticalTotalForceWithGround(
               this->mModel->GetLinkCollider(mSwingFoot->GetId())),
           stance_force = manger->GetVerticalTotalForceWithGround(
               this->mModel->GetLinkCollider(mStanceFoot->GetId()));
    if (true ==
        mStates[mStateIndex]->NeedTransition(mPhi, swing_force, stance_force))
    {
        int new_index = mStates[mStateIndex]->GetNextStateIndex();
        std::cout << "[log] transite to new state " << new_index << std::endl;
        TransiteToState(new_index);
    }
}

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
        auto state = new btGenSimbiconState(state_json, i, mModel);
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
    mRootId = 0;
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
    BTGEN_ASSERT(forces.size() == mModel->GetNumOfJoint());
    for (auto &x : forces)
    {
#ifndef APPLY_ROOT_TORQUE
        if (x.mJoint->GetIsRootJoint() == false)
#endif
        {
            std::cout << "[debug] joint " << x.mJoint->GetName()
                      << " force = " << x.mForce.transpose() << std::endl;
            mModel->ApplyJointTorque(x.mJoint->GetId(), x.mForce);

            // // draw control forces
            // {
            //     tVector3d st = x.mJoint->GetWorldPos();
            //     tVector3d ed = x.mForce.segment(0, 3) / 100;
            //     DrawLine(st, ed);
            // }
        }
    }
}

/**
 * \brief               Transite to a new state
 * \param state         state index
*/
void btGenSimbiconControllerBase::TransiteToState(int state)
{
    SetFSMStateTo(state);
    SetStance(mStates[this->mStateIndex]->GetStateStance(mStance));
    mPhi = 0;
}

/**
 * \brief           set the fsm state to antoher value
*/
void btGenSimbiconControllerBase::SetFSMStateTo(int state_idx)
{
    BTGEN_ASSERT(state_idx >= 0 && state_idx < mStates.size());
    mStateIndex = state_idx;
    // SetStance(mStates[mStateIndex]->GetStateStance(mStance));
}

/**
 * \brief           set the new stance configuration
*/
void btGenSimbiconControllerBase::SetStance(int new_stance)
{
    mStance = new_stance;

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

void btGenSimbiconControllerBase::UpdateRefModel(const tVectorXd &tar_pose)
{
    tVectorXd new_pose = tar_pose;
    new_pose[1] = 2.0;
    mRefTrajModel->SetqAndqdot(new_pose,
                               tVectorXd::Zero(mModel->GetNumOfFreedom()));
}