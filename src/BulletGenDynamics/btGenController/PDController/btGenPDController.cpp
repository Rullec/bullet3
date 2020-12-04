#include "btGenPDController.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <iostream>

btGenPDController::btGenPDController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::PDController, world)
{
    mEnablePDForceTest = false;
    mTargetq.resize(0);
    mTargetqdot.resize(0);
    mExpJointPDControllers.clear();
}

btGenPDController::~btGenPDController()
{
    for (auto &x : mExpJointPDControllers)
        delete x;
    mExpJointPDControllers.clear();
}

void btGenPDController::Init(cRobotModelDynamics *model,
                             const std::string &config)
{
    btGenControllerBase::Init(model, config);

    // 1. parse config
    ParseConfig(config);
}

/**
 * \brief               Set the PD target theta (gen coord)
 * \param q             all local coordinate target q (it will be converted to world frame automatically in CalcContorlForces)
 */
void btGenPDController::SetPDTargetq(const tVectorXd &q) { mTargetq = q; }

/**
 * \brief               Set the PD target vel (gen vel)
*/
void btGenPDController::SetPDTargetqdot(const tVectorXd &qdot)
{
    mTargetqdot = qdot;
}

/**
 * \brief           Update the PD Controller
 * 
*/
void btGenPDController::Update(double dt)
{
    // 1. calculate control forces
    tEigenArr<btGenPDForce> forces;
    CalculateControlForces(dt, forces);

    // 2. apply the control force
    BTGEN_ASSERT(forces.size() == mModel->GetNumOfJoint());
    for (int i = 0; i < forces.size(); i++)
    {
        auto joint = forces[i].mJoint;
        if (joint->GetIsRootJoint())
            continue;
        std::cout << "[pd] apply joint " << joint->GetId()
                  << " force = " << forces[i].mForce.transpose() << std::endl;
        mModel->ApplyJointTorque(joint->GetId(), forces[i].mForce);
    }
    std::cout << "[pd] pose err = " << (mTargetq - mModel->Getq()).transpose()
              << std::endl;
}
void btGenPDController::Reset() {}

/**
 * \brief       Parse the config of Gen PD controller
*/
void btGenPDController::ParseConfig(const std::string &string)
{
    Json::Value root;
    BTGEN_ASSERT(btJsonUtil::LoadJson(string, root));
    mEnableSPD = btJsonUtil::ParseAsBool("enable_stable_pd", root);
    mEnablePDForceTest = btJsonUtil::ParseAsBool("enable_pd_test", root);
    if (mEnablePDForceTest && mEnableSPD)
    {
        std::cout << "[error] PD force test is not available for SPD\n";
        exit(0);
    }

    // load the PD controlelrs, and check the completeness
    Json::Value joint_pd_controllers =
        btJsonUtil::ParseAsValue("PDControllers", root);
    BTGEN_ASSERT(mModel != nullptr);
    int num_of_joint = this->mModel->GetNumOfJoint();
    BTGEN_ASSERT(joint_pd_controllers.size() == num_of_joint);

    tVectorXd init_target_q = tVectorXd::Zero(mModel->GetNumOfFreedom());
    tVectorXd init_target_qdot = tVectorXd::Zero(mModel->GetNumOfFreedom());
    for (auto &single : joint_pd_controllers)
    {
        int joint_id = btJsonUtil::ParseAsInt("ID", single);
        std::string joint_name = btJsonUtil::ParseAsString("Name", single);
        double kp = btJsonUtil::ParseAsDouble("Kp", single),
               kd = btJsonUtil::ParseAsDouble("Kd", single),
               target_theta_init =
                   btJsonUtil::ParseAsDouble("TargetTheta0", single);
        bool use_world_coord = btJsonUtil::ParseAsBool("UseWorldCoord", single);
        Joint *cur_joint =
            dynamic_cast<Joint *>(mModel->GetJointById(joint_id));

        // keep the input order is ascending
        BTGEN_ASSERT(joint_id == mExpJointPDControllers.size());
        auto new_ctrl =
            new btGenJointPDCtrl(mModel, cur_joint, kp, kd,
                                 cur_joint->GetTorqueLim(), use_world_coord);
        init_target_q
            .segment(cur_joint->GetOffset(), cur_joint->GetNumOfFreedom())
            .fill(target_theta_init);
        init_target_qdot
            .segment(cur_joint->GetOffset(), cur_joint->GetNumOfFreedom())
            .fill(0);
        mExpJointPDControllers.push_back(new_ctrl);
        printf("[log] joint %d kp %.1f kd %.1f, torque lim %.1f, world_coord "
               "%s \n",
               joint_id, kp, kd, cur_joint->GetTorqueLim(),
               use_world_coord ? "true" : "false");
    }

    // init_target_q << 0, 0.75, 0, 0, 0, 0, -1.09805, 0, 0, 1.6, 0, 0, 0, 0, 0, 0,
    //     0, 0, 0, 0;
    printf("[error] the init target is set to random now!\n");
    init_target_q.setRandom();
    init_target_qdot.setZero();
    std::cout << "[pd] init q target = " << init_target_q.transpose()
              << std::endl;
    SetPDTargetq(init_target_q);
    SetPDTargetqdot(init_target_qdot);
}

/**
 * \brief           Calculate the PD control force
 * \param dt        time step
 * \param pd_forces forces
 * 
 * 1. Given the original target "mTargetq" and "mTargetqdot", calcualte the control target
 * 2. 
*/
void btGenPDController::CalculateControlForces(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    // 1. calculate the final target q
    tVectorXd target_q_use = CalcTargetPose(mTargetq);
    tVectorXd target_qdot_use = CalcTargetVel(mTargetqdot);

    if (mEnableSPD)
    {
        CalculateControlForcesSPD(dt, target_q_use, target_qdot_use, pd_forces);
    }
    else
    {
        CalculateControlForcesExp(target_q_use, target_qdot_use, pd_forces);
        if (mEnablePDForceTest == true)
        {
            std::cout << "[debug] PD controller test is enabled\n";
            TestPDController(dt);
        }
    }
}

/**
 * \brief           SPD implemention the same as raw SPD, but root Kp = 0
 * For more details please check the note, velocity is forced to be zero in this implemention, no gravity consideration
 * root Kp is not zero, the Gen force is set to zero at last
*/
void btGenPDController::CalculateControlForcesSPD(
    double dt, const tVectorXd &control_target_q,
    const tVectorXd &control_target_qdot, tEigenArr<btGenPDForce> &pd_forces)
{
    // 1. calculate SPD gen force by the formula
    int dof = mModel->GetNumOfFreedom();
    if (dof != control_target_q.size() || dof != control_target_qdot.size())
    {
        std::cout << "[error] "
                     "btGenPDController::CalculateControlForcesSPD "
                     "target q "
                     "and qdot has no correct size\n";
        exit(0);
    }
    tVectorXd q_cur = mModel->Getq(), qdot_cur = mModel->Getqdot();
    tVectorXd q_next_err = control_target_q - (q_cur + dt * qdot_cur);
    tVectorXd qdot_next_err = control_target_qdot - qdot_cur;

    tVectorXd Kp = tVectorXd::Zero(dof), Kd = tVectorXd::Zero(dof);
    for (int i = 0; i < this->mExpJointPDControllers.size(); i++)
    {
        auto &joint_ctrl = mExpJointPDControllers[i];
        const auto &joint = joint_ctrl->GetJoint();
        Kp.segment(joint->GetOffset(), joint->GetNumOfFreedom())
            .fill(joint_ctrl->GetKp());
        Kd.segment(joint->GetOffset(), joint->GetNumOfFreedom())
            .fill(joint_ctrl->GetKd());
        if (joint->GetIsRootJoint() == true)
        {
            Kp.segment(joint->GetOffset(), joint->GetNumOfFreedom()).setZero();
            Kd.segment(joint->GetOffset(), joint->GetNumOfFreedom()).setZero();
            printf("[debug] btGenPDController:CalculateControlForcesSPD "
                   "root Kp and Kd has been set to "
                   "zero in SPD calculation\n");
        }
    }

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kp_mat = Kp.asDiagonal();
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kd_mat = Kd.asDiagonal();

    // tVectorXd Q =
    //     mKp.cwiseProduct(q_next_err) + mKd.cwiseProduct(qdot_next_err);
    // std::cout << "Q = " << Q.transpose() << std::endl;
    tMatrixXd M = mModel->GetMassMatrix();
    M += dt * Kd_mat;

    tVectorXd qddot_pred =
        M.inverse() * (Kp_mat * q_next_err + Kd_mat * qdot_next_err -
                       mModel->GetCoriolisMatrix() * qdot_cur +
                       mModel->CalcGenGravity(mWorld->GetGravity()));
    // std::cout << "qddot pred = " << qddot_pred.transpose() << std::endl;
    tVectorXd tau =
        Kp_mat * q_next_err + Kd_mat * (qdot_next_err - dt * qddot_pred);
    BTGEN_ASSERT(tau.hasNaN() == false);

    // 2. convert the gen force to individual joint forces
    tEigenArr<tVector3d> joint_forces;
    tVector3d root_force, root_torque;
    mModel->ConvertGenForceToCartesianForceTorque(tau, joint_forces, root_force,
                                                  root_torque);
    // 3. convert joint forces to pd forces
    BTGEN_ASSERT(joint_forces.size() == mModel->GetNumOfJoint() - 1);

    pd_forces.clear();
    btGenPDForce pd_force;
    for (int i = 0; i < mModel->GetNumOfJoint(); i++)
    {
        pd_force.mJoint = this->mExpJointPDControllers[i]->GetJoint();
        if (i == 0)
        {
            pd_force.mForce = btMathUtil::Expand(root_torque, 0);
        }
        else
        {
            pd_force.mForce = btMathUtil::Expand(joint_forces[i - 1], 0);
            BTGEN_ASSERT(pd_force.mJoint->GetId() == i);
        }
        pd_forces.push_back(pd_force);
    }
}
/**
 * \brief           Calculate control forces by normal PD
 * \param 
*/
void btGenPDController::CalculateControlForcesExp(
    const tVectorXd &control_target_q, const tVectorXd control_target_qdot,
    tEigenArr<btGenPDForce> &pd_force_array)
{
    pd_force_array.clear();
    btGenPDForce pd_force;
    for (int i = 0; i < mExpJointPDControllers.size(); i++)
    {
        const auto &pd_ctrl = mExpJointPDControllers[i];
        pd_force.mForce =
            pd_ctrl->CalcControlForce(control_target_q, control_target_qdot);
        pd_force.mJoint = pd_ctrl->GetJoint();
        pd_force_array.push_back(pd_force);
    }
}

/**
 * \brief           Given generalized coordinate target q, calculate the gen coordinate target q w.r.t each joint
*/
tVectorXd btGenPDController::CalcTargetPose(const tVectorXd &tar_q) const
{
    tVectorXd control_target_q = tVectorXd::Zero(tar_q.size());
    for (auto &ctrl : mExpJointPDControllers)
    {
        auto joint = ctrl->GetJoint();
        int offset = joint->GetOffset(), size = joint->GetNumOfFreedom();
        control_target_q.segment(offset, size) =
            ctrl->CalcJointTargetPose(tar_q);
    }
    return control_target_q;
}

/**
 * \brief           Given gen vel tar_qdot, calculate the local control target qdot
*/
tVectorXd btGenPDController::CalcTargetVel(const tVectorXd &tar_qdot) const
{
    tVectorXd control_target_qdot = tVectorXd::Zero(tar_qdot.size());
    for (auto &ctrl : mExpJointPDControllers)
    {
        auto joint = ctrl->GetJoint();
        int offset = joint->GetOffset(), size = joint->GetNumOfFreedom();
        control_target_qdot.segment(offset, size) =
            ctrl->CalcJointTargetVel(tar_qdot);
    }
    return control_target_qdot;
}

std::vector<btGenJointPDCtrl *> &btGenPDController::GetJointPDCtrls()
{
    return mExpJointPDControllers;
}

/**
 * \brief           Test the PD controller
*/
void btGenPDController::TestPDController(double dt)
{
    std::cout << "-------------\n";
    // 1. test target pose
    TestPDControllerBuildPose(dt);
    // 2. test other norm cases
    TestPDControllerKpForce(dt);
    TestPDControllerKdForce(dt);
}
void btGenPDController::TestPDControllerBuildPose(double dt)
{
    // printf("[debug] test pd controller build pose begin\n");
    // 1. Test use world coordinate
    tVectorXd control_target = CalcTargetPose(mTargetq);
    for (auto &ctrl : mExpJointPDControllers)
    {
        if (true == ctrl->GetUseWorldCoord())
        {
            auto joint = ctrl->GetJoint();
            // 2. get the current gen coordinate q
            // tVectorXd cur_pose_all_local = mModel->Getq();
            // std::cout << "cur pose local = " << cur_pose_all_local.transpose()
            //           << std::endl;

            // 3. build the target local pose which can form the world orientation
            // tVectorXd target_pose_form_world_target = CalcTargetPose(mTargetq);
            // std::cout << "control target = "
            //           << target_pose_form_world_target.transpose() << std::endl;
            // std::cout << "build target q = "
            //           << target_pose_form_world_target.transpose() << std::endl;

            // 4. get the ideal world orientation
            mModel->PushState("test");
            mModel->SetqAndqdot(mTargetq,
                                tVectorXd::Zero(mModel->GetNumOfFreedom()));
            tMatrix3d ideal_orientation = joint->GetWorldOrientation();
            // std::cout << "raw target = " << mTargetq.transpose() << std::endl;
            // std::cout << "ideal orientation = \n"
            //           << ideal_orientation << std::endl;
            mModel->PopState("test");

            // 5. get the target world orientation by current local pose, and then verify
            {
                // 5.1 fetch the local pose of the target angle
                int offset = joint->GetOffset();
                int dof = joint->GetNumOfFreedom();
                // std::cout << "joint offset = " << offset << " dof = " << dof
                //           << std::endl;
                tVectorXd cur_pose = mModel->Getq();
                // std::cout << "local euler target = "
                //           << local_euler_target.transpose() << std::endl;
                // 5.2 put the local pose into the current pose
                cur_pose.segment(offset, dof) =
                    control_target.segment(offset, dof);
                // std::cout << "cur pose all local = "
                //           << cur_pose_all_local.transpose() << std::endl;
                // 5.3 set the cur pose all local, and get the joint world orientation again
                mModel->PushState("test");
                mModel->SetqAndqdot(cur_pose,
                                    tVectorXd::Zero(mModel->GetNumOfFreedom()));
                tMatrix3d pred_world_orientation = joint->GetWorldOrientation();
                mModel->PopState("test");

                // 5.4 judge that the pred & ideal is the same
                tMatrix3d diff = pred_world_orientation - ideal_orientation;
                // std::cout << "[debug] joint " << joint->GetId() << " diff = \n"
                //           << diff << std::endl;
                double norm = diff.norm();
                if (norm > 1e-6)
                {
                    std::cout << "joint " << joint->GetId() << std::endl;
                    std::cout << "pred world orientation = \n"
                              << pred_world_orientation << std::endl;
                    std::cout << "ideal orientation = \n"
                              << ideal_orientation << std::endl;
                    std::cout << "diff = \n" << diff << std::endl;
                    std::cout << "diff norm = " << norm << std::endl;
                    BTGEN_ASSERT(false);
                }
            }
            printf("[debug] TestPDController BuildPose for joint %d when "
                   "use_world_coord "
                   "= true, succ\n",
                   joint->GetId());
        }
    }
    printf("[debug] TestPDController BuildPose succ\n");
}

/**
 * \brief           verify the orientation of pd control force of Kp and q error
 *      is equal to the global orientation diff
*/
void btGenPDController::TestPDControllerKpForce(double dt)
{
    // push the Kd coeff, push the model state
    // 1. make all Kd = 0
    std::vector<double> kd_lst(0);
    for (auto &ctrl : mExpJointPDControllers)
    {
        kd_lst.push_back(ctrl->GetKd());
        ctrl->SetKd(0);
    }
    mModel->PushState("test");

    // 1.5 get the PD control forces
    tVectorXd control_target_q = CalcTargetPose(mTargetq);
    tVectorXd control_target_qdot = CalcTargetVel(mTargetqdot);
    tEigenArr<btGenPDForce> pd_forces;
    CalculateControlForcesExp(control_target_q, control_target_qdot, pd_forces);
    BTGEN_ASSERT(pd_forces.size() == mModel->GetNumOfJoint());

    int num_of_joints = mModel->GetNumOfJoint();
    BTGEN_ASSERT(pd_forces.size() == num_of_joints);
    // 2. test for each joint's controller
    for (auto &ctrl : mExpJointPDControllers)
    {
        // 3. calculate the desired world orientation
        tMatrix3d desired_rot = tMatrix3d::Zero();
        Joint *joint = ctrl->GetJoint();
        int joint_id = joint->GetId();
        int offset = joint->GetOffset(), size = joint->GetNumOfFreedom();
        if (ctrl->GetUseWorldCoord() == true)
        {
            // if use the world orientation
            mModel->PushState("get_ideal_orient");
            mModel->SetqAndqdot(mTargetq, mTargetqdot);
            desired_rot = joint->GetWorldOrientation();
            mModel->PopState("get_ideal_orient");
        }
        else
        {
            // if don't use the world orientation
            tVectorXd cur_q = mModel->Getq();
            cur_q.segment(offset, size) = mTargetq.segment(offset, size);
            mModel->PushState("get_ideal_orient");
            mModel->SetqAndqdot(cur_q, mModel->Getqdot());
            desired_rot = joint->GetWorldOrientation();
            mModel->PopState("get_ideal_orient");
        }

        // 4. calculate the current orientation
        tMatrix3d cur_rot = joint->GetWorldOrientation();

        // 5. verify
        if (std::fabs(ctrl->GetKp()) < 1e-10)
        {
            printf("[error] joint %d kp = %.3f will cause the failure of "
                   "verification\n",
                   joint->GetId(), ctrl->GetKp());
            BTGEN_ASSERT(false);
        }
        auto verify_rotation_pd_force = [](const tMatrix3d &desired_rot,
                                           const tMatrix3d &cur_rot,
                                           const tVector &force, int joint_id) {
            tVector aa = btMathUtil::RotmatToAxisAngle(
                btMathUtil::ExpandMat(desired_rot * cur_rot.transpose(), 0));
            tVector diff_1 = force.normalized() - aa.normalized();
            tVector diff_2 = force.normalized() + aa.normalized();
            tVector diff = diff_1.norm() > diff_2.norm() ? diff_2 : diff_1;
            if (diff.norm() > 1e-5)
            {
                std::cout << "joint id = " << joint_id << std::endl;
                std::cout << "desired_rot = \n" << desired_rot << std::endl;
                std::cout << "cur = \n" << cur_rot << std::endl;
                std::cout << "aa = " << aa.transpose() << std::endl;
                std::cout << "pd force = " << force.transpose() << std::endl;
                BTGEN_ASSERT(false);
            }
        };
        // doesn't verify for root joint
        if (joint->GetIsRootJoint() == true)
        {
            switch (joint->GetJointType())
            {
            case JointType::NONE_JOINT:
            {
                verify_rotation_pd_force(desired_rot, cur_rot,
                                         pd_forces[joint_id].mForce, joint_id);
                break;
            }
            case JointType::BIPEDAL_NONE_JOINT:
            {
                verify_rotation_pd_force(desired_rot, cur_rot,
                                         pd_forces[joint_id].mForce, joint_id);
                break;
            }
            case JointType::LIMIT_NONE_JOINT:
            {
                tVector f(ctrl->GetKp() * (mTargetq[0] - mModel->Getq()[0]), 0,
                          0, 0);
                BTGEN_ASSERT((f - pd_forces[joint_id].mForce).norm() < 1e-5);
                break;
            }
            case JointType::FIXED_NONE_JOINT:
            {
                BTGEN_ASSERT(pd_forces[joint_id].mForce.norm() < 1e-10);
                break;
            }

            default:
                BTGEN_ASSERT(false);
                break;
            }
        }
        else
        {
            // if not root joint
            // desired = diff * cur
            // diff = desired * cur.inv
            verify_rotation_pd_force(desired_rot, cur_rot,
                                     pd_forces[joint_id].mForce, joint_id);
        }
    }

    // pop the Kd coeff, pop the model state
    for (int i = 0; i < mExpJointPDControllers.size(); i++)
    {
        btGenJointPDCtrl *ctrl = mExpJointPDControllers[i];
        ctrl->SetKd(kd_lst[i]);
    }

    std::cout << "[debug] TestPDController KpForce succ\n";
    mModel->PopState("test");
}
/**
 * \brief               verify the orientation of pd control force of Kd and qdot error
 *          is equal to the orientation of joint angular velocity,
*/
void btGenPDController::TestPDControllerKdForce(double dt)
{
    // push the Kd coeff, push the model state
    // 1. make all Kp = 0
    std::vector<double> kp_lst(0);
    for (auto &ctrl : mExpJointPDControllers)
    {
        kp_lst.push_back(ctrl->GetKp());
        ctrl->SetKp(0);
    }
    mModel->PushState("test");

    tVectorXd control_target_qdot = CalcTargetVel(mTargetqdot);
    tVectorXd control_target_q = CalcTargetPose(mTargetq);

    tEigenArr<btGenPDForce> pd_forces;
    CalculateControlForcesExp(control_target_q, control_target_qdot, pd_forces);
    int num_of_joints = mModel->GetNumOfJoint();
    BTGEN_ASSERT(pd_forces.size() == num_of_joints);

    // 2. calculate the control force
    for (auto &ctrl : mExpJointPDControllers)
    {
        // 3. calculate the desired world velocity
        Joint *joint = ctrl->GetJoint();
        int joint_id = joint->GetId();
        mModel->PushState("get_ideal_vel");
        tVectorXd target_qdot_joint = mTargetqdot;
        target_qdot_joint.segment(0, joint->GetOffset()).setZero();
        tVector3d desired_vel = joint->GetJKw() * target_qdot_joint;
        mModel->PopState("get_ideal_vel");

        // 4. calculate the current orientation
        tVectorXd qdot = mModel->Getqdot();
        qdot.segment(0, joint->GetOffset()).setZero();
        tVector3d cur_vel = joint->GetJKw() * qdot;

        if (std::fabs(ctrl->GetKd()) < 1e-10)
        {
            printf("[error] joint %d kd = %.3f will cause the failure of "
                   "verification\n",
                   joint->GetId(), ctrl->GetKd());
            BTGEN_ASSERT(false);
        }
        // 5. verify
        // doesn't verify the root
        auto verify = [](const tVector3d &desired_vel, const tVector3d &cur_vel,
                         const tVector &force, int joint_id) {
            // desired_vel - cur_vel
            tVector3d ideal_force_dir = (desired_vel - cur_vel).normalized();
            tVector3d diff = force.normalized().segment(0, 3) -

                             ideal_force_dir;
            BTGEN_ASSERT(diff.norm() < 1e-5);
        };
        if (joint->GetIsRootJoint() == true)
        {
            switch (joint->GetJointType())
            {
            case JointType::NONE_JOINT:
            case JointType::BIPEDAL_NONE_JOINT:
            {
                verify(desired_vel, cur_vel, pd_forces[joint_id].mForce,
                       joint_id);
                break;
            }
            case JointType::LIMIT_NONE_JOINT:
            {
                // verify the velocity result, only has a x axis velocity movement
                tVector3d tar_root_linvel = joint->GetJKv() * mTargetqdot;
                tVector3d cur_root_linvel = joint->GetJKv() * mModel->Getqdot();
                tVector force = btMathUtil::Expand(
                    (tar_root_linvel - cur_root_linvel) * ctrl->GetKd(), 0);
                BTGEN_ASSERT((force - pd_forces[joint_id].mForce).norm() <
                             1e-10);
                break;
            }
            case JointType::FIXED_NONE_JOINT:
            {
                BTGEN_ASSERT(pd_forces[joint_id].mForce.norm() < 1e-5);
            }
            default:
                BTGEN_ASSERT(false);
                break;
            }
        }
        else
        {
            // not root joint, verify as usuall
            verify(desired_vel, cur_vel, pd_forces[joint_id].mForce, joint_id);
        }
    }
    // pop the Kp coeff, pop the model state
    for (int i = 0; i < mExpJointPDControllers.size(); i++)
    {
        btGenJointPDCtrl *ctrl = mExpJointPDControllers[i];
        ctrl->SetKp(kp_lst[i]);
    }
    std::cout << "[debug] TestPDController KdForce succ\n";
    mModel->PopState("test");
}