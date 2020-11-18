#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconController.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSM.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSMUtil.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"

btGenSimbiconController::btGenSimbiconController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::SimbiconController, world)
{
    mRootId = 0;
    this->mSwingHip = -1;
    mStanceHip = -1;
    mFSM = nullptr;
    mPDController = nullptr;
    mCd = 0;
    mCv = 0;
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
    // 2. build FSM
    BuildFSM(fsm_config);
    // 3. build & correct PD controller
    BuildPDCtrl(pd_ctrl_path);
    // 4. build the balance policy
    BuildBalanceCtrl(balance_ctrl);

    // 5. init pose
    {
        mFSM->InitPose();
    }

    // 6. create ref char
    {
        mRefTrajModel = new cRobotModelDynamics();

        mRefTrajModel->Init(mModel->GetCharFile().c_str(), mModel->GetScale(),
                            ModelType::JSON);
        mRefTrajModel->InitSimVars(mWorld->GetInternalWorld(), true, true,
                                   false);

        mRefTrajModel->SetqAndqdot(mModel->Getq(), mModel->Getqdot());
    }
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

    // update target pose
    UpdateRefModel();

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
        if (mSwingHip == -1 || mStanceHip == -1)
        {
            printf("[log] swing hip or stance hip is -1, disable the inverse "
                   "solution of hip torque\n");
        }
        else
        {
            tVector torso_torque = pd_forces[this->mRootId].mForce;
            // std::cout << "torso torque = " << torso_torque.transpose()
            //           << std::endl;

            tVector swing_torque = pd_forces[this->mSwingHip].mForce;
            // std::cout << "swing torque = " << swing_torque.transpose()
            //           << std::endl;
            tVector stance_torque = -torso_torque - swing_torque;
            pd_forces[mStanceHip].mForce = stance_torque;
        }

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
 * The simbicon's PD controler doesn't work as usual, 
*/
void btGenSimbiconController::BuildPDCtrl(const std::string &pd_path)
{
    mPDController = new btGenPDController(mWorld);
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
 * \brief           Build the balance control policy
*/
void btGenSimbiconController::BuildBalanceCtrl(const Json::Value &conf)
{
    this->mCd = btJsonUtil::ParseAsDouble("Cd", conf);
    this->mCv = btJsonUtil::ParseAsDouble("Cv", conf);
}

/**
 * \brief               Update the target pose by the blance control
 * \param target_pose   target q got from the FSM
 * 1. find the swing hip, 
 * 2. get the COM pos "d" & vel "v" 
 * 3. change the control target of swing hip by theta = theta_d + c_d * d + c_v * v
*/
void btGenSimbiconController::BalanceUpdateTargetPose(
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
    auto swing_hip = dynamic_cast<Joint *>(mModel->GetJointById(mSwingHip));
    BTGEN_ASSERT(swing_hip->GetJointType() == JointType::REVOLUTE_JOINT);

    int offset = swing_hip->GetOffset();
    int size = swing_hip->GetNumOfFreedom();
    BTGEN_ASSERT(size == 1);
    double theta = target_pose[offset];
    tVector3d com_pos = mModel->GetCoMPosition(),
              com_vel = mModel->GetComVelocity();
    double d = com_pos[2] - mModel->GetJointById(GetEndeffector(mStanceHip))
                                ->GetWorldPos()[2],
           v = com_vel[2];

    // minus angle means uplift, for current hips
    target_pose[offset] = -mCd * d - mCv * v + theta;

    printf("[simbicon] d = %.3f, v = %.3f, origin theta %.3f, "
           "result theta %.3f for swing hip %s\n",
           d, v, theta, target_pose[offset], swing_hip->GetName().c_str());
}

/**
 * \brief           Update the swing and stance hip's id (also the root id)
 * 1. find two hips
 * 2. find their end effector (two feet)
 * 3. check contact for these two feet
 * 4. confirm the swing hip and stance hip (exit in the illegal case)
*/
#include "BulletGenDynamics/btGenController/SimbiconController/FSMUtil.h"
void btGenSimbiconController::UpdateSwingStance()
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
    bool left_foot_contact =
        mWorld->GetTwoObjsNumOfContact(
            mWorld->GetGround(), mModel->GetLinkCollider(left_foot_id)) > 0;
    bool right_foot_contact =
        mWorld->GetTwoObjsNumOfContact(
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
    {
        if (left_foot_contact == false && right_foot_contact == false)
        {
            printf(
                "[simbicon] illegal case: left and right foot contact case is "
                "the same, keep the current swing %d stance %d\n",
                mSwingHip, mStanceHip);

            return;
        }
    }

    //
    if (right_foot_contact == true)
        mSwingHip = left_hip_id, mStanceHip = right_hip_id;
    else
        mStanceHip = left_hip_id, mSwingHip = right_hip_id;
}

/**
 * \brief           Get the end effector id for given joint
 * \param id        given joint id
 * \return          the id of end effector  
*/
int btGenSimbiconController::GetEndeffector(int id) const
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
void btGenSimbiconController::UpdatePDController(const tVectorXd &tar_pose)
{
    auto ctrls = this->mPDController->GetJointPDCtrls();
    for (auto &x : ctrls)
    {
        x->SetUseWorldCoord(false);

        int id = x->GetJoint()->GetId();
        if (id == this->mRootId || id == mSwingHip)
        {
            // printf("[log] Set joint %d PD control use world coord\n", id);
            x->SetUseWorldCoord(true);
        }
    }

    std::cout << "[log] final PD target = " << tar_pose.transpose()
              << std::endl;
    mPDController->SetPDTargetq(tar_pose);
}

/**
 * \brief           Restrict the root's rootation to X axis, root's translation in YOZ plane
 *              
*/
void btGenSimbiconController::RestrictRootPose()
{
    BTGEN_ASSERT(mModel->GetJointById(0)->GetJointType() ==
                 JointType::NONE_JOINT);
    tVectorXd q = mModel->Getq(), qdot = mModel->Getqdot();
    q[0] = 0;                  // translation: YOZ plane
    q.segment(4, 2).setZero(); // rotation: only x axis
    qdot[0] = 0;
    qdot.segment(4, 2).setZero();
    mModel->SetqAndqdot(q, qdot);
}

/**
 * \brief           Judge if the character fall down
*/
bool btGenSimbiconController::IsFallDown() const
{
    int num = mWorld->GetTwoObjsNumOfContact(mWorld->GetGround(),
                                             mModel->GetLinkCollider(mRootId));
    return num > 0;
}

/**
 * \brief           Update the pose of ref model (debugging purpose)
*/
void btGenSimbiconController::UpdateRefModel()
{
    BaseObject *joint = mRefTrajModel->GetJointById(0);
    int move_dof_id = -1;
    if (joint->GetJointType() == JointType::NONE_JOINT)
        move_dof_id = 3;
    else if (joint->GetJointType() == JointType::BIPEDAL_NONE_JOINT)
        move_dof_id = 0;
    double move_distance = 1;
    tVectorXd tar_pose = mFSM->GetTargetPose();

    tar_pose[move_dof_id] += move_distance;
    mRefTrajModel->SetqAndqdot(tar_pose, mRefTrajModel->Getqdot());
}