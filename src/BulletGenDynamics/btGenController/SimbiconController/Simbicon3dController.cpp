#include "Simbicon3dController.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSM.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSMUtil.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include <iostream>
btGenSimbicon3dController::btGenSimbicon3dController(btGeneralizeWorld *world)
    : btGenSimbiconControllerBase(world,
                                  ebtGenControllerType::Simbicon3dController)
{
}
btGenSimbicon3dController::~btGenSimbicon3dController() {}
void btGenSimbicon3dController::Init(cRobotModelDynamics *model,
                                     const std::string &conf)
{
    btGenSimbiconControllerBase::Init(model, conf);

    // confirm this model is in 3d but not bipedal
    Joint *root = dynamic_cast<Joint *>(
        mModel->GetJointById(GetJointByPartialName("root")));
    Joint *left_hip = dynamic_cast<Joint *>(
        mModel->GetJointById(GetJointByPartialName("LeftLeg")));
    Joint *right_hip = dynamic_cast<Joint *>(
        mModel->GetJointById(GetJointByPartialName("RightLeg")));

    BTGEN_ASSERT(root->GetJointType() == JointType::NONE_JOINT);
    BTGEN_ASSERT(left_hip->GetJointType() == JointType::SPHERICAL_JOINT);
    BTGEN_ASSERT(right_hip->GetJointType() == JointType::SPHERICAL_JOINT);
}
void btGenSimbicon3dController::Update(double dt)
{
    printf("-------------simbicon update cur time %.3f cur state "
           "%d--------------\n",
           mTime, mFSM->GetCurrentState()->GetStateId());

    btGenControllerBase::Update(dt);
    if (IsFallDown() == true)
    {
        printf("[log] character fall down, simbicon finished\n");
        exit(0);
    }
    // 0. make the root stay in YOZ plane and has no X translation
    // RestrictRootPose();
    // 0. update hips
    UpdateSwingStance();

    // 1. update FSM, get the target pose
    tVectorXd target_pose;
    mFSM->Update(dt, target_pose);

    // 2. change the target pose by balance control policy
    BalanceUpdateTargetPose(target_pose);

    // 3. set the target into the PD controller
    {
        // 3.1 find the swing hip and root, set the use world coord to true
        UpdatePDController(target_pose);
        // 3.2 calculate the control force (including root)
        tEigenArr<btGenPDForce> pd_forces(0);
        mPDController->CalculateControlForces(dt, pd_forces);
        BTGEN_ASSERT(pd_forces.size() == mModel->GetNumOfJoint());

        /*
            3.3 calculate the control force for the stance hip by inverse method

            \tau_A = stance_hip_torque
            \tau_B = swing_hip_torque
            \tau_torso = torso_torque
            \tau_A = -\tau_torso - \tau_B
        */
        // if (mSwingHip == -1 || mStanceHip == -1)
        // {
        //     printf("[log] swing hip or stance hip is -1, disable the inverse "
        //            "solution of hip torque\n");
        // }
        // else
        // {
        //     tVector torso_torque = pd_forces[this->mRootId].mForce;
        //     // std::cout << "torso torque = " << torso_torque.transpose()
        //     //           << std::endl;

        //     tVector swing_torque = pd_forces[this->mSwingHip].mForce;
        //     // std::cout << "swing torque = " << swing_torque.transpose()
        //     //           << std::endl;
        //     tVector stance_torque = -torso_torque - swing_torque;
        //     pd_forces[mStanceHip].mForce = stance_torque;
        // }

        // 3.4 apply the control force (except root)
        for (auto &x : pd_forces)
        {
            int joint_id = x.mJoint->GetId();
            if (joint_id == 0)
                continue;
            else
            {
                mModel->ApplyJointTorque(joint_id, x.mForce);
                // std::cout << "[simbicon] joint " << joint_id
                //           << " torque = " << x.mForce.transpose() << std::endl;
            }
        }
    }

    // update target pose
    UpdateRefModel(target_pose);
}
void btGenSimbicon3dController::Reset() {}

/**
 * \brief           Build the balance control parameter
*/
void btGenSimbicon3dController::BuildBalanceCtrl(const Json::Value &conf)
{
    mCd_forward = btJsonUtil::ParseAsDouble("Cd_forward", conf);
    mCv_forward = btJsonUtil::ParseAsDouble("Cv_forward", conf);
    mCd_tanget = btJsonUtil::ParseAsDouble("Cd_tanget", conf);
    mCv_tanget = btJsonUtil::ParseAsDouble("Cv_tanget", conf);
}

/**
 * \brief           balance control policy for 3d simbicon controller
 * 
 *      Adjust the target pose of swing hip
 * 
 * 1. for forward direction balance control
 *          let d is the distance from stance feet to COM
 *          if d>0, make the swing hip higher (x rotation smaller)
 *          if d<0, make the swing hip lower (x rotation bigger)
 *
 * 2. for horizontal (tangential) direction balance control, (keep balance for left and right swing)
 *      2.1 way1: let d is the distance from stance feet to COM, along with X
 *                  |Y+
 *                  |
 *                  |
 *    Right hip     |   Left hip
 *                  |
 *                  --------------------X+    
 *          if d>0, swing hip = right hip, right foot move right (z rotation bigger)
 *                  swing hip = left hip, left foot move right (z rotation bigger)
 *          if d<0, swing hip = right hip, right foot move left (z rotation smaller)
 *                  swing hip = left hip, left foot move left (z rotation smaller)
 *
 *      2.2 way2: let d is the distance from zero to COM
 *          other rules are the same
*/
void btGenSimbicon3dController::BalanceUpdateTargetPose(
    tVectorXd &target_pose) const
{
    if (mSwingHip == -1 || mStanceHip == -1)
    {
        printf("[warn] swing/stance hip is -1, balance control disabled\n");
        return;
    }
    auto cur_state = mFSM->GetCurrentState();
    if (mIgnoreBalanceControlInState02 == true &&
        cur_state->GetStateId() != 1 && cur_state->GetStateId() != 3)
    {
        printf("[warn] balance control is ignored in state id %d temporarily\n",
               cur_state->GetStateId());
        return;
    }

    // 1. get the d and v
    tVector3d d, v;
    auto swing_hip = dynamic_cast<Joint *>(mModel->GetJointById(mSwingHip));
    {
        BTGEN_ASSERT(swing_hip->GetJointType() == JointType::SPHERICAL_JOINT);
        tVector3d com_pos = mModel->GetCoMPosition(),
                  com_vel = mModel->GetComVelocity();
        std::cout << "[debug] com pos = " << com_pos.transpose()
                  << " com vel = " << com_vel.transpose() << std::endl;
        auto stance_foot_id = GetEndeffector(mStanceHip);
        auto stance_foot =
            dynamic_cast<Joint *>(mModel->GetJointById(stance_foot_id));
        tVector3d stance_foot_pos = stance_foot->GetWorldPos();
        d = com_pos - stance_foot_pos, v = com_vel;
    }

    // 2. convert current target to axis angle
    int offset = swing_hip->GetOffset();
    int size = swing_hip->GetNumOfFreedom();
    BTGEN_ASSERT(size == 3);
    tVector3d target_euler_xyz = target_pose.segment(offset, size);

    tVector3d target_axisangle =
        btMathUtil::EulerangleToAxisAngle(
            btMathUtil::Expand(target_euler_xyz, 0), btRotationOrder::bt_XYZ)
            .segment(0, 3);

    double target_x = target_axisangle[0], target_z = target_axisangle[2];
    // 3. adjust x rotation by forward balance control
    {
        std::cout << "[debug] origin target x = " << target_x << std::endl;
        double d_forward = d[2], v_forward = v[2];
        target_x += -mCd_forward * d_forward - mCv_forward * v_forward;
        std::cout << "[debug] after target x = " << target_x << std::endl;
    }
    // 4. adjust z rotation by horizontal balance control
    {
        std::cout << "[debug] origin target z = " << target_z << std::endl;
        double d_tanget = d[0], v_tanget = v[0];
        target_z += mCd_tanget * d_tanget + mCv_tanget * v_tanget;
        std::cout << "[debug] after target z = " << target_z << std::endl;
    }

    // 5. print out and write back
    std::cout << "[debug] origin axis angle = " << target_axisangle.transpose()
              << std::endl;
    target_axisangle[0] = target_x;
    target_axisangle[2] = target_z;
    std::cout << "[debug] after axis angle = " << target_axisangle.transpose()
              << std::endl;
    tVector new_target_euler_xyz = btMathUtil::AxisAngleToEulerAngle(
        btMathUtil::Expand(target_axisangle, 0), btRotationOrder::bt_XYZ);
    std::cout << "[debug] origin euler xyz = " << target_euler_xyz.transpose()
              << std::endl;
    std::cout << "[debug] after euler xyz = "
              << new_target_euler_xyz.transpose() << std::endl;
    target_pose.segment(offset, size) = new_target_euler_xyz.segment(0, 3);
}

/**
 * \brief           Update Simbicon SPD controller
 *      1. 
 *      2. 
*/
#include "BulletGenDynamics/btGenController/PDController/btGenSimbiconSPDController.h"
void btGenSimbicon3dController::UpdatePDController(const tVectorXd &tar_pose)
{
    // do nothing
    printf("[warn] do nothing in UpdatePDController\n");
}