#include "btGenPDController.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include <iostream>

btGenPDController::btGenPDController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::PDController, world)
{
    mTargetqSet.resize(0);
    mTargetqdotSet.resize(0);
    mTargetqCur.resize(0);
    mTargetqdotCur.resize(0);
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
void btGenPDController::SetPDTargetq(const tVectorXd &q)
{
    mTargetqSet = q;

    // if (mEnableSPD == false)
    // {
    //     // if SPD is disabled, dispatch the target q to each joint
    //     int num_of_freedom = mModel->GetNumOfFreedom();
    //     BTGEN_ASSERT(mTargetq.size() == num_of_freedom);
    //     tVectorXd old_q = mModel->Getq();
    //     mModel->Apply(mTargetq, false);
    //     for (auto pd_ctrl : mExpJointPDControllers)
    //     {
    //         auto joint = pd_ctrl->GetJoint();
    //         if (pd_ctrl->GetUseWorldCoord() == false)
    //         {
    //             pd_ctrl->SetTargetTheta(q);
    //         }
    //         else
    //         {
    //             printf("[debug] for joint %d, use world coord PD control, "
    //                    "begin to set PD target q\n",
    //                    joint->GetId());
    //             // get the target world rotation of this joint
    //             // convert it to quaternion, then set it to the controller
    //             pd_ctrl->SetTargetTheta(btMathUtil::QuaternionToCoef(
    //                 btMathUtil::RotMat3dToQuaternion(
    //                     joint->GetWorldOrientation())));
    //             std::cout << "world target = "
    //                       << pd_ctrl->GetTargetTheta().transpose() << std::endl;
    //         }
    //     }
    //     mModel->Apply(old_q, false);
    // }
}

/**
 * \brief               Set the PD target vel (gen vel)
*/
void btGenPDController::SetPDTargetqdot(const tVectorXd &qdot)
{
    mTargetqdotSet = qdot;

    // if (mEnableSPD == false)
    // {
    //     // if SPD is disabled, dispatch the target qdot to each joint's PD Controller
    //     int num_of_freedom = mModel->GetNumOfFreedom();
    //     BTGEN_ASSERT(mTargetqdot.size() == num_of_freedom);
    //     for (auto &pd_ctrl : mExpJointPDControllers)
    //     {
    //         auto joint = pd_ctrl->GetJoint();
    //         pd_ctrl->SetTargetVel(qdot);
    //     }
    // }
}

// /**
//  * \brief               Calculate the generalized control torque and apply it to the robot model
//  */
// void btGenPDController::ApplyGeneralizedTau(double timestep)
// {
//     int dof = mModel->GetNumOfFreedom();
//     if (dof != mTargetq.size() || dof != mTargetqdot.size())
//     {
//         std::cout << "[error] btGenPDController::ApplyGeneralizedTau target q "
//                      "and qdot hasn't been set\n";
//         exit(0);
//     }
//     tVectorXd tau = tVectorXd::Zero(dof);

//     tVectorXd q = mModel->Getq(), qdot = mModel->Getqdot(),
//               qddot = mModel->Getqddot();
//     std::cout << "qdot = " << qdot.transpose() << std::endl;
//     std::cout << "qddot = " << qddot.transpose() << std::endl;
//     for (int i = 0; i < dof; i++)
//     {
//         // std::cout << "qdot " << i << " = " << qdot[i] << ", max vel = " <<
//         // mModel->GetMaxVel() << std::endl; std::cout << "res = " <<
//         // (std::fabs(qdot[i]) - 10) << std::endl;
//         if ((std::fabs(qdot[i])) >= mModel->GetMaxVel() - 5)
//         {
//             std::cout << "qdot " << i << " = " << qdot[i]
//                       << " exceed the maxvel, belongs to joint "
//                       << mModel->GetJointByDofId(i)->GetName() << std::endl;
//         }
//     }
//     if (mEnableSPD == false)
//     {
//         tau = mKp.cwiseProduct(mTargetq - q) +
//               mKd.cwiseProduct(mTargetqdot - qdot);
//     }
//     else
//     {
//         tau = -mKp.cwiseProduct(q + timestep * qdot - mTargetq) -
//               mKd.cwiseProduct(qdot + timestep * qddot - mTargetqdot);
//     }

//     double max_tau = tau.cwiseAbs().maxCoeff();
//     if (max_tau > mTorqueLim)
//     {
//         std::cout << "the computed control torque " << max_tau << " > "
//                   << mTorqueLim << ", clamped!";
//         std::cout << "raw tau = " << tau.transpose() << std::endl;
//         tau = tau.cwiseMax(-mTorqueLim);
//         tau = tau.cwiseMin(mTorqueLim);
//     }

//     // underactuated: the control torque on root joint is prohibeted
//     tau.segment(0, 6).setZero();

//     for (int i = 0; i < dof; i++)
//         mModel->ApplyGeneralizedForce(i, tau[i]);

//     std::cout << "PD controller apply tau = " << tau.transpose() << std::endl;
// }

/**
 * \brief           Update the PD Controller
 * 
*/
void btGenPDController::Update(double dt)
{
    // printf("pd controller update dt %.5f\n", dt);

    // 2. calculate pd forces and apply
    tEigenArr<btGenPDForce> forces;
    CalculateControlForces(dt, forces);

    // apply the control force
    BTGEN_ASSERT(forces.size() == mModel->GetNumOfJoint());
    for (int i = 0; i < forces.size(); i++)
    {
        auto joint = forces[i].mJoint;
        std::cout << "[pd] apply joint " << joint->GetId()
                  << " force = " << forces[i].mForce.transpose() << std::endl;
        mModel->ApplyJointTorque(joint->GetId(), forces[i].mForce);
    }
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

    SetPDTargetq(init_target_q);
    SetPDTargetqdot(init_target_qdot);
}

/**
 * \brief           Calculate the control force by PD control
 * \param dt        time step
 * \param pd_forces forces
*/
void btGenPDController::CalculateControlForces(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    // 1. build target pose (from joints), change the target pose if some controllers are using world coordinate
    mTargetqCur = mTargetqSet;
    mTargetqdotCur = mTargetqdotSet;
    BuildTargetPose(mTargetqCur);
    BuildTargetPose(mTargetqdotCur);

    if (mEnableSPD)
        CalculateControlForcesSPD(dt, pd_forces);
    else
        CalculateControlForcesExp(pd_forces);
}

/**
 * \brief           Calcualte control forces by SPD (stable PD)
 * For more details please check the note, velocity is forced to be zero in this implemention
 * 
*/
void btGenPDController::CalculateControlForcesSPD(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{

    // 1. calculate SPD gen force by the formula
    int dof = mModel->GetNumOfFreedom();
    if (dof != mTargetqCur.size() || dof != mTargetqdotCur.size())
    {
        std::cout
            << "[error] btGenPDController::CalculateControlForcesSPD target q "
               "and qdot has no correct size\n";
        exit(0);
    }
    tVectorXd q_cur = mModel->Getq(), qdot_cur = mModel->Getqdot();
    tVectorXd q_next_err = mTargetqCur - (q_cur + dt * qdot_cur);
    tVectorXd qdot_next_err = mTargetqdotCur - qdot_cur;

    tVectorXd Kp = tVectorXd::Zero(dof), Kd = tVectorXd::Zero(dof);
    for (int i = 0; i < this->mExpJointPDControllers.size(); i++)
    {
        auto &joint_ctrl = mExpJointPDControllers[i];
        const auto &joint = joint_ctrl->GetJoint();
        Kp.segment(joint->GetOffset(), joint->GetNumOfFreedom())
            .fill(joint_ctrl->GetKp());
        Kd.segment(joint->GetOffset(), joint->GetNumOfFreedom())
            .fill(joint_ctrl->GetKd());
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
                       mModel->GetCoriolisMatrix() * qdot_cur);
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
    tEigenArr<btGenPDForce> &pd_force_array)
{
    pd_force_array.clear();
    btGenPDForce pd_force;
    for (int i = 0; i < mExpJointPDControllers.size(); i++)
    {
        const auto &pd_ctrl = mExpJointPDControllers[i];
        pd_force.mForce =
            pd_ctrl->CalcControlForce(mTargetqCur, mTargetqdotCur);
        pd_force.mJoint = pd_ctrl->GetJoint();
        pd_force_array.push_back(pd_force);
    }
}

/**
 * \brief           convert the given fully local target pose, to a revised, local target pose which part of them are in world coordinate
*/
void btGenPDController::BuildTargetPose(tVectorXd &pose)
{
    for (auto &ctrl : mExpJointPDControllers)
    {
        ctrl->BuildTargetPose(pose);
    }
}
void btGenPDController::BuildTargetVel(tVectorXd &vel)
{
    for (auto &ctrl : mExpJointPDControllers)
    {
        ctrl->BuildTargetPose(vel);
    }
}

std::vector<btGenJointPDCtrl *> &btGenPDController::GetJointPDCtrls()
{
    return mExpJointPDControllers;
}