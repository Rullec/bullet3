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
    btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::SimbiconController, world)
{
    mFSM = nullptr;
    mPDController = nullptr;
    mRefTrajModel = nullptr;
    mStance = -1;
    mRootId = 0;
    mCd_forward = 0;
    mCv_forward = 0;
    mCd_tangent = 0;
    mCv_tangent = 0;
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
    mEnableDrawHeadingFrame =
        btJsonUtil::ParseAsBool("enable_draw_heading_frame", root);

    mInitPose =
        btJsonUtil::ReadVectorJson(btJsonUtil::ParseAsValue("init_pose", root));

    std::string init_stance = btJsonUtil::ParseAsString("init_stance", root);
    int init_state = btJsonUtil::ParseAsInt("init_state", root);
    if (init_stance == "left")
        mStance = BTGEN_LEFT_STANCE;
    else if (init_stance == "right")
        mStance = BTGEN_RIGHT_STANCE;
    else
        BTGEN_ASSERT(false);

    BuildJointInfo();

    // 2. build FSM
    BuildFSM(fsm_config);
    // 3. build & correct PD controller
    BuildPDCtrl(pd_ctrl_path);
    // 4. build the balance policy
    BuildBalanceCtrl(balance_ctrl);

    // 5. initialzethe FSM
    // set the model pose + set the default state + set the default stance
    mFSM->Init();
    UpdateStance();

    // 7. set init pose
    BTGEN_ASSERT(mInitPose.size() == mModel->GetNumOfFreedom());
    mModel->SetqAndqdot(mInitPose, tVectorXd::Zero(mModel->GetNumOfFreedom()));
    mFSM->SetState(init_state);
    std::cout << "[simbicon] set init pose = " << mInitPose.transpose()
              << " init stance = " << init_stance
              << " init state = " << init_state << std::endl;

    // 6. create ref char
    mRefTrajModel = new cRobotModelDynamics();

    mRefTrajModel->Init(mModel->GetCharFile().c_str(), mModel->GetScale(),
                        ModelType::JSON);
    mRefTrajModel->InitSimVars(mWorld, true, true, false);

    UpdateRefModel(mModel->Getq());
}

/**
 * \brief           Simbicon build blance control policy (2 coeffs)
*/
void btGenSimbiconControllerBase ::BuildBalanceCtrl(const Json::Value &conf)
{
    mCd_forward = btJsonUtil::ParseAsDouble("Cd_forward", conf);
    mCv_forward = btJsonUtil::ParseAsDouble("Cv_forward", conf);
    mCd_tangent = btJsonUtil::ParseAsDouble("Cd_tangent", conf);
    mCv_tangent = btJsonUtil::ParseAsDouble("Cv_tangent", conf);
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
    // 1. update FSM, get the target pose
    tVectorXd target_pose;
    mFSM->Update(dt, target_pose, mStance, mSwingFootInfo.first,
                 mStanceFootInfo.first);
    UpdateStance();

    // 2. change the target pose by balance control policy
    CalcTargetPose(target_pose);

    // 3. set the target into the PD controller
    // find the swing hip and root, set the use world coord to true
    UpdatePDController(target_pose);

    // 4. calculate control force
    tEigenArr<btGenPDForce> pd_forces(0);
    CalcControlForce(dt, pd_forces);
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
            printf("[error] simbicon PD controller joint %d force lim %.3f is "
                   "too small!",
                   x->GetJoint()->GetId(), x->GetForceLim());
            exit(0);
        }
    }
}
/**
 * \brief           Update the swing and stance hip's id (also the root id)
*/
void btGenSimbiconControllerBase::UpdateStance()
{
    if (mStance == BTGEN_LEFT_STANCE)
    {
        printf("[simbicon] swing: right, stance: left\n");
        mStanceHipInfo = mLeftHipInfo;
        mStanceFootInfo = mLeftFootInfo;
        mSwingHipInfo = mRightHipInfo;
        mSwingFootInfo = mRightFootInfo;
    }
    else if (mStance == BTGEN_RIGHT_STANCE)
    {
        printf("[simbicon] swing: left, stance: right\n");
        mStanceHipInfo = mRightHipInfo;
        mStanceFootInfo = mRightFootInfo;
        mSwingHipInfo = mLeftHipInfo;
        mSwingFootInfo = mLeftFootInfo;
    }
    else
    {
        BTGEN_ASSERT(false && "invalid stance value");
    }
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
        if (id == mRootId || id == mSwingHipInfo.first)
        {
            printf("[log] Set joint %s PD control use world coord\n",
                   x->GetJoint()->GetName().c_str());
            x->SetUseWorldCoord(true);
        }
    }

    // std::cout << "[log] final PD target = " << tar_pose.transpose()
    //           << std::endl;

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
double btGenSimbiconControllerBase::CalcStanceSwingRatio() const
{
    // 1. get stance foot and swing foot
    int stance_foot_id = mStanceFootInfo.first,
        swing_foot_id = mSwingFootInfo.first;

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
void btGenSimbiconControllerBase::CalcControlForce(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    mPDController->CalculateControlForces(dt, pd_forces);
    BTGEN_ASSERT(pd_forces.size() == mModel->GetNumOfJoint());

    tVector torso_torque = pd_forces[this->mRootId].mForce,
            swing_torque = pd_forces[mSwingHipInfo.first].mForce,
            stance_torque = pd_forces[mStanceHipInfo.first].mForce;

    tVector new_stance_torque = tVector::Zero();
    if (true == mEnableStanceControlRatio)
    {
        double stance_swing_foot_ratio = CalcStanceSwingRatio();

        std::cout << "[log] stance ratio = " << stance_swing_foot_ratio
                  << ", torso torque = " << torso_torque.transpose()
                  << std::endl;

        std::cout << "[debug] origin swing torque = "
                  << swing_torque.transpose()
                  << " stance torque = " << stance_torque.transpose()
                  << std::endl;

        // \tau_makeup = \tau_torso - \tau_swing - \tau_stance

        tVector torso_makeup_torque = -torso_torque;

        // old method
        // {
        //     torso_makeup_torque -= -swing_torque - stance_torque;
        // }

        // new method
        {
            for (int i = 0; i < mModel->GetNumOfJoint(); i++)
            {
                auto joint = mModel->GetJointById(i);
                if (mRootId == joint->GetParentId())
                {
                    // std::cout << "[debug] joint " << i << " "
                    //           << joint->GetName()
                    //           << " is root's child, included in torso makeup "
                    //              "torque\n";
                    torso_makeup_torque -= pd_forces[i].mForce;
                }
            }
        }

        // if (mModel->GetRoot()->GetJointType() == JointType::BIPEDAL_NONE_JOINT)
        // {
        //     BTGEN_ASSERT(std::fabs(torso_torque[1]) < 1e-10);
        //     BTGEN_ASSERT(std::fabs(torso_torque[2]) < 1e-10);
        //     // \tau_makeup = \tau_torso - \tau_swing - \tau_stance
        //     torso_makeup_torque.segment(1, 3).setZero();
        // }
        // \tau_swing += (1-k) * \tau_makeup
        swing_torque += (1 - stance_swing_foot_ratio) * torso_makeup_torque;
        // \tau_stance += k * \tau_makeup
        stance_torque += stance_swing_foot_ratio * torso_makeup_torque;
        pd_forces[mSwingHipInfo.first].mForce = swing_torque;
        pd_forces[mStanceHipInfo.first].mForce = stance_torque;
        std::cout << "[debug] new swing torque = " << swing_torque.transpose()
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
        pd_forces[mStanceHipInfo.first].mForce = new_stance_torque;
    }
}

/**
 * \brief           Update the target pose by COM vel and COM pos
*/
void btGenSimbiconControllerBase::CalcTargetPoseRevoluteHips(
    tVectorXd &target_pose)
{
    auto swing_hip =
        dynamic_cast<Joint *>(mModel->GetJointById(mSwingHipInfo.first));
    int offset = swing_hip->GetOffset();
    int size = swing_hip->GetNumOfFreedom();
    BTGEN_ASSERT(size == 1);
    double theta = target_pose[offset];
    tVector3d com_pos = mModel->GetCoMPosition(),
              com_vel = mModel->GetComVelocity();
    double d = com_pos[2] -
               mModel->GetJointById(GetEndeffector(mStanceHipInfo.first))
                   ->GetWorldPos()[2],
           v = com_vel[2];

    // minus angle means uplift, for current hips
    target_pose[offset] = -mCd_forward * d - mCv_forward * v + theta;

    printf("[simbicon] d = %.3f, v = %.3f, origin theta %.3f, "
           "result theta %.3f for swing hip %s\n",
           d, v, theta, target_pose[offset], swing_hip->GetName().c_str());
}

/**
 * \brief           Calculate Feedback target pose for spherical swing hip
 * 1. calculate the heading frame's X and Y axis's expressions in world frame, x_head_world and y_head_world
 * 2. calculate the vector d and v in world frame, then convert them to heading frame, get d_head and v_head
 * 3. caluclate the forward and tangent joint angle feedback value e_{forward} and e_{tangent} in R
 * 4.   e_{forward} + x_head_world = forward_target_feedback, in joint rest frame
 *      e_{tangent} + y_head_world = tangent_target_feedback, in joint rest frame
 * 5. apply the feedback to the local target
*/
void btGenSimbiconControllerBase::CalcTargetPoseSphericalHips(
    tVectorXd &target_pose)
{
    // 1. heading frame's x and z axis in world frame
    auto swing_joint =
        dynamic_cast<Joint *>(mModel->GetJointById(mSwingHipInfo.first));
    tVector x_head_world = tVector(1, 0, 0, 0),
            y_head_world = tVector(0, 1, 0, 0);
    tMatrix heading_rotmat = tMatrix::Identity();
    {
        double heading_angle = mModel->GetHeading();
        tVector heading_aa = tVector(0, heading_angle, 0, 0);
        heading_rotmat = btMathUtil::AxisAngleToRotmat(heading_aa);
        x_head_world = heading_rotmat * x_head_world;
        y_head_world = heading_rotmat * y_head_world;
        // std::cout << "[debug] x_head_world = " << x_head_world.transpose()
        //           << " y_head_world = " << y_head_world.transpose()
        //           << std::endl;
        // std::cout << "[debug] heading rotmat = \n"
        //           << heading_rotmat << std::endl;
    }

    // 2. calculate d and v in world frame, then convert to heading frame
    tVector d_head = tVector::Zero(), v_head = tVector::Zero();
    {
        // 2.1 d = com_pos - stance_foot_pos
        d_head.segment(0, 3) =
            mModel->GetCoMPosition() -
            mModel->GetLinkById(mStanceFootInfo.first)->GetWorldPos();
        d_head[1] = 0; // remove Y axis effect

        // 2.2 v = com_vel
        v_head.segment(0, 3) = mModel->GetComVelocity();
        v_head[1] = 0; // remove Y axis effect
        // std::cout << "[debug] d_world = " << d_head.transpose()
        //           << " v_world = " << v_head.transpose() << std::endl;

        // convert to heading frame
        d_head = heading_rotmat.transpose() * d_head;
        v_head = heading_rotmat.transpose() * v_head;
        // std::cout << "[debug] d_head = " << d_head.transpose()
        //           << " v_head = " << v_head.transpose() << std::endl;
    }

    // 3. calculate e_forward and e_tanget
    double e_forward = 0, e_tangent = 0;
    {
        e_forward =
            -mCd_forward * d_head[2] -
            mCv_forward *
                v_head
                    [2]; // forward feedback effected by z axis translation and velocity
        e_tangent =
            mCd_tangent * d_head[0] +
            mCv_tangent *
                v_head
                    [0]; // tangent feedback effectred by x axis translation and velocity
        // std::cout << "[debug] e_forward = " << e_forward
        //           << " e_tangent = " << e_tangent << std::endl;
    }

    // 4. get the forward and tagent feedback rotation (in axis angle form)
    tVector forward_feedback_local = tVector::Zero(),
            tangent_feedback_local = tVector::Zero();
    {
        tVector forward_feedback_world = x_head_world * e_forward,
                tangent_feedback_world = y_head_world * e_tangent;

        // world = rest * local
        // rest = world * local.T
        // joint_world_to_local = rest.T = local * world.T
        tMatrix3d joint_local_rotation =
                      swing_joint->GetLocalTransform().block(0, 0, 3, 3),
                  joint_world_rotation = swing_joint->GetWorldOrientation();
        tMatrix3d joint_world_to_local =
            joint_local_rotation * joint_world_rotation.transpose();
        // std::cout << "[debug] joint world to local = \n"
        //           << joint_world_to_local << std::endl;
        forward_feedback_local.segment(0, 3) =
            joint_world_to_local * forward_feedback_world.segment(0, 3);
        tangent_feedback_local.segment(0, 3) =
            joint_world_to_local * tangent_feedback_world.segment(0, 3);
        // std::cout << "[debug] forward feedback local = "
        //           << forward_feedback_local.transpose()
        //           << " tangent feedback local = "
        //           << tangent_feedback_local.transpose() << std::endl;
    }

    // 5. apply these two feedback axis angle
    {
        tVector raw_local_target_euler = tVector::Zero();
        raw_local_target_euler.segment(0, 3) = target_pose.segment(
            swing_joint->GetOffset(), swing_joint->GetNumOfFreedom());
        tMatrix raw_local_target_rotmat = btMathUtil::EulerAnglesToRotMat(
            raw_local_target_euler, btRotationOrder::bt_XYZ);
        tMatrix feedback =
            btMathUtil::AxisAngleToRotmat(forward_feedback_local) *
            btMathUtil::AxisAngleToRotmat(tangent_feedback_local);
        tMatrix new_local_target_rotmat = feedback * raw_local_target_rotmat;
        tVector new_local_target_euler = btMathUtil::RotmatToEulerAngle(
            new_local_target_rotmat, btRotationOrder::bt_XYZ);
        target_pose.segment(swing_joint->GetOffset(),
                            swing_joint->GetNumOfFreedom()) =
            new_local_target_euler.segment(0, 3);
        // std::cout << "[debug] new swing local target = "
        //           << new_local_target_euler.transpose() << std::endl;
    }
}

/**
 * \brief               Update the target pose by the blance control
 * \param target_pose   target q got from the FSM
 * 1. find the swing hip, 
 * 2. get the COM pos "d" & vel "v" 
 * 3. change the control target of swing hip by theta = theta_d + c_d * d + c_v * v
*/
void btGenSimbiconControllerBase::CalcTargetPose(tVectorXd &target_pose)
{
    auto cur_state = mFSM->GetCurrentState();
    if (mIgnoreBalanceControlInState02 == true &&
        cur_state->GetStateId() != 1 && cur_state->GetStateId() != 3)
    {
        printf("[warn] balance control is ignored in state id %d temporarily\n",
               cur_state->GetStateId());
        return;
    }

    auto swing_hip =
        dynamic_cast<Joint *>(mModel->GetJointById(mSwingHipInfo.first));

    switch (swing_hip->GetJointType())
    {
    case JointType::REVOLUTE_JOINT:
        CalcTargetPoseRevoluteHips(target_pose);
        break;
    case JointType::SPHERICAL_JOINT:
        CalcTargetPoseSphericalHips(target_pose);
        break;
    default:
        BTGEN_ASSERT(false);
        break;
    };
}

/**
 * \brief               Get the swing hip info and right 
 * 1. find two hips
 * 2. find their end effector (two feet)
*/
void btGenSimbiconControllerBase::BuildJointInfo()
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
    mLeftHipInfo.first = left_hip_id;
    mLeftHipInfo.second = mModel->GetLinkById(left_hip_id)->GetName();
    mRightHipInfo.first = right_hip_id;
    mRightHipInfo.second = mModel->GetLinkById(right_hip_id)->GetName();
    printf("[simbicon] left hip %d %s, right hip %d %s\n", left_hip_id,
           mLeftHipInfo.second.c_str(), right_hip_id,
           mRightHipInfo.second.c_str());
    // judge one chain: left hip and right hip should have only one chain
    // get the end effector of this chain
    int left_foot_id = GetEndeffector(left_hip_id),
        right_foot_id = GetEndeffector(right_hip_id);

    mLeftFootInfo.first = left_foot_id;
    mLeftFootInfo.second = mModel->GetLinkById(left_foot_id)->GetName();
    mRightFootInfo.first = right_foot_id;
    mRightFootInfo.second = mModel->GetLinkById(right_foot_id)->GetName();
    printf("[simbicon] left foot %d %s, right foot %d %s\n", left_foot_id,
           mLeftFootInfo.second.c_str(), right_foot_id,
           mRightFootInfo.second.c_str());
}

/**
 * \brief               verify the current heading frame (Y axis) is correct
*/
void btGenSimbiconControllerBase::DebugVerifyHeadingFrame()
{
    std::cout << "[warn] verify heading frame enabled\n";
    double heading = mModel->GetHeading();
    tVector aa = tVector(0, heading, 0, 0);
    tMatrix trans = btMathUtil::AxisAngleToRotmat(aa);
    tVector forward_vec = tVector(1, 0, 0, 0);
    tVector head_vec = trans * forward_vec;
    tVector model_vec =
        btMathUtil::ExpandMat(mModel->GetLinkById(0)->GetWorldOrientation(), 0) *
        forward_vec;
    model_vec[1] = 0;
    model_vec.normalize();
    head_vec.normalize();
    tVector diff = head_vec - model_vec;
    if (diff.norm() > 1e-5)
    {
        std::cout << "[debug] verify heading frame, diff = " << diff.transpose()
                  << std::endl;
        std::cout << "[error] heading = " << heading << std::endl;
        std::cout << "[error] head_vec = " << head_vec.transpose() << std::endl;
        std::cout << "[error] model_vec = " << model_vec.transpose()
                  << std::endl;
        exit(0);
    }
}

/**
 * \brief               Draw the heading frame in the current result 
*/
void btGenSimbiconControllerBase::DrawHeadingFrame()
{
    double heading = mModel->GetHeading(); // heading axis and Y transform
    tVector aa = tVector(0, heading, 0, 0);
    tMatrix trans = btMathUtil::AxisAngleToRotmat(aa);
    trans.block(0, 3, 3, 1) =
        mModel->GetLinkById(0)->GetWorldPos().segment(0, 3);
    printf("[log] draw heading frame begin, angle = %.4f\n", heading);
    DrawFrame(trans);
    // tMatrix trans = mModel->GetLinkById(0)->GetGlobalTransform();
    // DrawFrame(trans);
    // exit(0);
}