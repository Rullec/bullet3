#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconControllerBase.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSM.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSMUtil.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenSolver/ContactManager.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"

btGenSimbiconControllerBase::btGenSimbiconControllerBase(
    btGeneralizeWorld *world, ebtGenControllerType type)
    : btGenControllerBase(type, world)
{
    mRootId = 0;
    this->mSwingHip = -1;
    mStanceHip = -1;
    mFSM = nullptr;
    mPDController = nullptr;
}

btGenSimbiconControllerBase::~btGenSimbiconControllerBase()
{
    delete mFSM;
    mFSM = nullptr;
    delete mPDController;
    mPDController = nullptr;
}

/**
 * \brief               Initialize the simbicon controller
*/
void btGenSimbiconControllerBase::Init(cRobotModelDynamics *model,
                                       const std::string &conf)
{
    btGenControllerBase::Init(model, conf);
    // 1. check the skeleton(fixed now)
    std::string char_file = model->GetCharFile();

    Json::Value root;
    btJsonUtil::LoadJson(conf, root);
    const Json::Value &fsm_config = btJsonUtil::ParseAsValue("FSM", root);
    std::string pd_ctrl_path =
        btJsonUtil::ParseAsString("pd_controller_path", root);
    const Json::Value &balance_ctrl =
        btJsonUtil::ParseAsValue("balance_control", root);
    std::string corresponding_char_path =
        btJsonUtil::ParseAsString("corresponding_char_path", root);
    BTGEN_ASSERT(char_file == corresponding_char_path);
    mIgnoreBalanceControlInState02 =
        btJsonUtil::ParseAsBool("ignore_balance_control_in_0_2_state", root);
    mEnableStanceControlRatio =
        btJsonUtil::ParseAsBool("enable_stance_control_ratio", root);

    // 2. build FSM
    BuildFSM(fsm_config);
    // 3. build & correct PD controller
    BuildPDCtrl(pd_ctrl_path);
    // 4. build the balance policy
    BuildBalanceCtrl(balance_ctrl);

    // 5. init pose
    mFSM->InitPose();

    // 6. create ref char
    mRefTrajModel = new cRobotModelDynamics();

    mRefTrajModel->Init(mModel->GetCharFile().c_str(), mModel->GetScale(),
                        ModelType::JSON);
    mRefTrajModel->InitSimVars(mWorld->GetInternalWorld(), true, true, false);

    mRefTrajModel->SetqAndqdot(mModel->Getq(), mModel->Getqdot());

    // {
    //     for (int i = 0; i < mModel->GetNumOfLinks(); i++)
    //     {
    //         auto link = dynamic_cast<Link *>(mModel->GetLinkById(i));
    //         BTGEN_ASSERT(link != nullptr);
    //         std::cout << "link " << i << " inertia = "
    //                   << link->GetInertiaTensorBody().diagonal().transpose()
    //                   << std::endl;
    //     }
    //     exit(0);
    // }
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
void btGenSimbiconControllerBase::Update(double dt)
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
    // find the swing hip and root, set the use world coord to true
    UpdatePDController(target_pose);

    // 4. calculate control force
    tEigenArr<btGenPDForce> pd_forces(0);
    CalculateControlForce(dt, pd_forces);
    for (auto &x : pd_forces)
    {
        int joint_id = x.mJoint->GetId();
        if (joint_id != 0)
        {
            mModel->ApplyJointTorque(joint_id, x.mForce);
        }
    }

    // update target pose
    UpdateRefModel(target_pose);
}

void btGenSimbiconControllerBase::Reset() {}

/**
 * \brief           Build FSM
*/
void btGenSimbiconControllerBase::BuildFSM(const Json::Value &conf)
{
    mFSM = new btGenFSM(mWorld, mModel, conf);
}
/**
 * \brief           Build the PD controller
 * The simbicon's PD controler doesn't work as usual, 
*/
#include "BulletGenDynamics/btGenController/BuildController.h"
void btGenSimbiconControllerBase::BuildPDCtrl(const std::string &pd_path)
{
    mPDController = dynamic_cast<btGenPDController *>(
        BuildController(this->mWorld, pd_path));
    mPDController->Init(mModel, pd_path);

    // check the PD coef
    for (auto &x : mPDController->GetJointPDCtrls())
    {
        if (x->GetForceLim() < 1)
        {
            printf("[error] simbicon PD controller joint %f force lim %.3f is "
                   "too small!",
                   x->GetJoint()->GetId(), x->GetForceLim());
            exit(0);
        }
    }
}
/**
 * \brief           Update the swing and stance hip's id (also the root id)
 * 1. find two hips
 * 2. find their end effector (two feet)
 * 3. check contact for these two feet
 * 4. confirm the swing hip and stance hip (exit in the illegal case)
*/
#include "BulletGenDynamics/btGenController/SimbiconController/FSMUtil.h"
void btGenSimbiconControllerBase::UpdateSwingStance()
{
    std::string left_hip_name = "LeftLeg", right_hip_name = "RightLeg";
    int left_hip_id = -1, right_hip_id = -1;
    for (int i = 0; i < mModel->GetNumOfLinks(); i++)
    {
        auto joint = mModel->GetJointById(i);
        if (joint->GetName().find(left_hip_name) != -1)
        {
            if (left_hip_id != -1)
            {
                printf("[error] left hip id get failed\n");
                exit(0);
            }
            else
            {
                left_hip_id = i;
            }
        }
        if (joint->GetName().find(right_hip_name) != -1)
        {
            if (right_hip_id != -1)
            {
                printf("[error] right hip id get failed\n");
                exit(0);
            }
            else
            {
                right_hip_id = i;
            }
        }
    }

    if (left_hip_id == -1 || right_hip_id == -1)
    {
        printf("[error] failed to get two hips: %d %d\n", left_hip_id,
               right_hip_id);
        exit(0);
    }

    // judge one chain: left hip and right hip should have only one chain
    // get the end effector of this chain
    int left_foot_id = GetEndeffector(left_hip_id),
        right_foot_id = GetEndeffector(right_hip_id);

    // then check whether this link is contacted with the ground
    auto manager = mWorld->GetContactManager();
    bool left_foot_contact =
        manager->GetTwoObjsNumOfContact(
            mWorld->GetGround(), mModel->GetLinkCollider(left_foot_id)) > 0;
    bool right_foot_contact =
        manager->GetTwoObjsNumOfContact(
            mWorld->GetGround(), mModel->GetLinkCollider(right_foot_id)) > 0;

    printf("[log] left foot contact %s, right foot contact %s\n",
           left_foot_contact ? "true" : "false",
           right_foot_contact ? "true" : "false");

    // if the configuration is wrong, set to -1
    // {
    //     if (left_foot_contact == right_foot_contact)
    //     {
    //         mStanceHip = -1;
    //         mSwingHip = -1;
    //         printf(
    //             "[simbicon] illegal case: left and right foot contact case is "
    //             "the same, set them to -1\n");
    //         return;
    //     }

    //     if (left_foot_contact == true)
    //         mStanceHip = left_hip_id, mSwingHip = right_hip_id;
    //     else
    //         mSwingHip = left_hip_id, mStanceHip = right_hip_id;
    // }

    // if the configuration is wrong, use the old configuration
    // {
    //     if (left_foot_contact == false && right_foot_contact == false)
    //     {
    //         printf(
    //             "[simbicon] illegal case: left and right foot contact case is "
    //             "the same, keep the current swing %d stance %d\n",
    //             mSwingHip, mStanceHip);
    //         return;
    //     }
    // }
    {
        if (left_foot_contact == false && right_foot_contact == false)
        {
            mStanceHip = -1;
            mSwingHip = -1;
            mStanceName = "";
            mSwingName = "";
            printf(
                "[simbicon] illegal case: left and right foot contact case is "
                "false, set them to -1\n");
            return;
        }
    }

    if (right_foot_contact == true)
        mSwingHip = left_hip_id, mStanceHip = right_hip_id;
    else
        mStanceHip = left_hip_id, mSwingHip = right_hip_id;
    mSwingName = mModel->GetLinkById(mSwingHip)->GetName();
    mStanceName = mModel->GetLinkById(mStanceHip)->GetName();
}

/**
 * \brief           Get the end effector id for given joint
 * \param id        given joint id
 * \return          the id of end effector  
*/
int btGenSimbiconControllerBase::GetEndeffector(int id) const
{
    auto cur_link = mModel->GetLinkById(id);
    int num_of_child = cur_link->GetNumOfChildren();

    while (num_of_child != 0)
    {
        if (num_of_child != 1)
        {
            printf("[error] link %d has %d children, illegal!\n", id,
                   num_of_child);
            exit(0);
        }

        // toward the child joint's child link
        cur_link = cur_link->GetFirstChild()->GetFirstChild();
        id = cur_link->GetId();
        num_of_child = cur_link->GetNumOfChildren();
    }
    return id;
}

/**
 * \brief           Update the PD Controller's UseWorldCoord Setting, and update the new 
 * 1. set the PD Controller root, swing hip, torso's to use world coordinate
 * 2. set the new PD target into the controller
*/
void btGenSimbiconControllerBase::UpdatePDController(const tVectorXd &tar_pose)
{
    auto ctrls = this->mPDController->GetJointPDCtrls();
    for (auto &x : ctrls)
    {
        x->SetUseWorldCoord(false);

        int id = x->GetJoint()->GetId();
        if (id == this->mRootId || id == mSwingHip)
        {
            printf("[log] Set joint %s PD control use world coord\n",
                   x->GetJoint()->GetName().c_str());
            x->SetUseWorldCoord(true);
        }
    }

    std::cout << "[log] final PD target = " << tar_pose.transpose()
              << std::endl;

    // std::cout << "[log] model q = " << mModel->Getq().transpose() << std::endl;
    // std::cout << "[log] model qdot = " << mModel->Getqdot().transpose()
    //           << std::endl;

    mPDController->SetPDTargetq(tar_pose);
}

/**
 * \brief           Judge if the character fall down
*/
bool btGenSimbiconControllerBase::IsFallDown() const
{
    int num = mWorld->GetContactManager()->GetTwoObjsNumOfContact(
        mWorld->GetGround(), mModel->GetLinkCollider(mRootId));
    return num > 0;
}

/**
 * \brief           Update the pose of ref model (debugging purpose)
*/
void btGenSimbiconControllerBase::UpdateRefModel(const tVectorXd &target_pose)
{
    // use the FSM tar pose
    // {BaseObject *joint = mRefTrajModel->GetJointById(0);
    // int move_dof_id = -1;
    // if (joint->GetJointType() == JointType::NONE_JOINT)
    //     move_dof_id = 1;
    // else if (joint->GetJointType() == JointType::BIPEDAL_NONE_JOINT)
    //     move_dof_id = 0;
    // double move_distance = 1;
    // tVectorXd tar_pose = mFSM->GetTargetPose();

    // tar_pose[move_dof_id] += move_distance;
    // mRefTrajModel->SetqAndqdot(tar_pose, mRefTrajModel->Getqdot());}

    // 2. use the revised target pose
    {
        tVectorXd set_tar_pose = target_pose;
        BaseObject *joint = mRefTrajModel->GetJointById(0);
        if (joint->GetJointType() == JointType::NONE_JOINT)
        {
            set_tar_pose.segment(0, 3).setZero();
            set_tar_pose[1] = 2;
        }
        else if (joint->GetJointType() == JointType::BIPEDAL_NONE_JOINT)
        {
            set_tar_pose.segment(0, 2).setZero();
            set_tar_pose[0] = 2;
        }

        mRefTrajModel->SetqAndqdot(set_tar_pose, mRefTrajModel->Getqdot());
    }
}

/**
 * \brief           Get the Joint by part of name
*/
int btGenSimbiconControllerBase::GetJointByPartialName(const std::string &name)
{
    int joint_id = -1;
    for (int i = 0; i < mModel->GetNumOfLinks(); i++)
    {
        auto cur_joint = mModel->GetJointById(i);
        if (cur_joint->GetName().find(name) != -1)
        {
            if (joint_id != -1)
            {
                printf("[error] left hip id get failed\n");
                exit(0);
            }
            else
            {
                joint_id = i;
            }
        }
    }
    BTGEN_ASSERT(joint_id != -1);
    return joint_id;
}

/**
 * \brief           Compute the stance and swing control vertical force ratio
 * \return double [0, 1], stance foot vertical force / total_vertical_force
*/
double btGenSimbiconControllerBase::ComputeStanceSwingRatio() const
{
    // 1. get stance foot and swing foot
    int stance_foot_id = GetEndeffector(mStanceHip),
        swing_foot_id = GetEndeffector(mSwingHip);

    auto manager = mWorld->GetContactManager();
    double stance_force = manager->GetVerticalTotalForceWithGround(
               mModel->GetLinkCollider(stance_foot_id)),
           swing_force = manager->GetVerticalTotalForceWithGround(
               mModel->GetLinkCollider(swing_foot_id));
    double total_force = stance_force + swing_force + 1e-6;
    double ratio = stance_force / total_force;
    printf("[log] Simbicon stance ratio: stance force %.3f, swing force %.3f, "
           "total force %.3f, ratio %.3f\n",
           stance_force, swing_force, total_force, ratio);

    return ratio;
}

/**
 * \brief               calculate PD control force
 * 1. caluclate the PD force
 * 2. change the stance hip PD force by revised policy (simple rule + ratio rule)
 * 3. apply these control forces
 * 
 * For more details, please check the note and raw simbicon implemention
*/
void btGenSimbiconControllerBase::CalculateControlForce(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    mPDController->CalculateControlForces(dt, pd_forces);
    BTGEN_ASSERT(pd_forces.size() == mModel->GetNumOfJoint());

    if (mSwingHip == -1 || mStanceHip == -1)
    {
        printf("[log] swing hip or stance hip is -1, disable the inverse "
               "solution of hip torque\n");
    }
    else
    {
        tVector torso_torque = pd_forces[this->mRootId].mForce,
                swing_torque = pd_forces[this->mSwingHip].mForce,
                stance_torque = pd_forces[this->mStanceHip].mForce;

        tVector new_stance_torque = tVector::Zero();
        if (true == mEnableStanceControlRatio)
        {
            double stance_swing_foot_ratio = ComputeStanceSwingRatio();

            std::cout << "[log] stance ratio = " << stance_swing_foot_ratio
                      << ", torso torque = " << torso_torque.transpose()
                      << std::endl;

            std::cout << "[debug] origin swing torque = "
                      << swing_torque.transpose()
                      << " stance torque = " << stance_torque.transpose()
                      << std::endl;

            // \tau_makeup = \tau_torso - \tau_swing - \tau_stance
            tVector torso_makeup_torque =
                -torso_torque - swing_torque - stance_torque;

            if (mModel->GetRoot()->GetJointType() ==
                JointType::BIPEDAL_NONE_JOINT)
            {
                BTGEN_ASSERT(std::fabs(torso_torque[1]) < 1e-10);
                BTGEN_ASSERT(std::fabs(torso_torque[2]) < 1e-10);
                // \tau_makeup = \tau_torso - \tau_swing - \tau_stance
                torso_makeup_torque.segment(1, 3).setZero();
            }
            // \tau_swing += (1-k) * \tau_makeup
            swing_torque += (1 - stance_swing_foot_ratio) * torso_makeup_torque;
            // \tau_stance += k * \tau_makeup
            stance_torque += stance_swing_foot_ratio * torso_makeup_torque;
            pd_forces[mSwingHip].mForce = swing_torque;
            pd_forces[mStanceHip].mForce = stance_torque;
            std::cout << "[debug] new swing torque = "
                      << swing_torque.transpose()
                      << " stance torque = " << stance_torque.transpose()
                      << std::endl;
        }
        else
        {
            /*
                    simple rule:
                    \tau_A = stance_hip_torque
                    \tau_B = swing_hip_torque
                    \tau_torso = torso_torque
                    \tau_A = -\tau_torso - \tau_B
            */
            new_stance_torque = -torso_torque - swing_torque;
            std::cout << "[log] stance hip torque simple covered = "
                      << new_stance_torque.transpose() << std::endl;
            pd_forces[mStanceHip].mForce = new_stance_torque;
        }
    }
}