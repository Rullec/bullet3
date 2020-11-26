#include "btGenSimbiconSPDController.h"
#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <iostream>

btGenSimbiconSPDController::btGenSimbiconSPDController(btGeneralizeWorld *world)
    : btGenPDController(world)
{
    mCtrlType = ebtGenControllerType::SimbiconSPDController;
    mJointIdUpdated = false;
    mTargetUpdated = false;
}

btGenSimbiconSPDController::~btGenSimbiconSPDController() {}
void btGenSimbiconSPDController::Init(cRobotModelDynamics *model,
                                      const std::string &config)

{
    btGenPDController::Init(model, config);
    std::cout << "simbicon SPD inited\n";
}
void btGenSimbiconSPDController::Update(double dt)
{
    CalcTargetPose(mTargetq);
    CalcTargetVel(mTargetq);
    mTargetUpdated = true;
}
void btGenSimbiconSPDController::Reset() {}

/**
 * \brief               Calculate control force from simbicon
 * 
 * if stance & swing hip id = -1, call normal SPD
 * else call simbicon-SPD
 * 
*/
void btGenSimbiconSPDController::CalculateControlForces(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    BTGEN_ASSERT(mJointIdUpdated && mTargetUpdated);

    if (EnableSimbiconSPD() == true)
    {
        BTGEN_ASSERT(JointType::SPHERICAL_JOINT ==
                     mModel->GetJointById(mSwingHipId)->GetJointType());
        BTGEN_ASSERT(JointType::SPHERICAL_JOINT ==
                     mModel->GetJointById(mStanceHipId)->GetJointType());
        CalculateControlForcesSimbiconSPD(dt, pd_forces);
    }
    else
    {
        CalculateControlForcesSPD(dt, pd_forces);
    }
    mTargetUpdated = false;
    mJointIdUpdated = false;
}

void btGenSimbiconSPDController::SetJointId(int swing_hip, int stance_hip,
                                            int root_id)
{
    mSwingHipId = swing_hip;
    mStanceHipId = stance_hip;
    mRootId = root_id;
    mJointIdUpdated = true;
}

/**
 * \brief           Calculate the simbicon-spd jointed control force
 * g = E * Kp * (q_tar - q_cur - dt * qdot_cur) + E  * Kd * (qdot_tar - qdot_cur) + f
 * qddot_pred = (M + dt * Kd).inv() * (g + QG - C * qdot_cur)
 * Q_tau' = E * Kp * (q_tar - q_cur - dt * qdot_cur) + E * Kd * (qdot_tar - qdot_cur - dt * qddot_pred) + f
 * 
 * J_stance = J_{stance_child} - J_{stance_parent}
 * 
 * We needs to calculate
 * 1. E = I, but in stance row, swing column, the value = -J_stance^T * (J_swing^T)^{-1}
 * 2. f = \vec{0}, but in stance row, the value = -J_stance^T * \tau_torso
 * 3. g
 * 4. qddot_pred
 * 5. Q_tau'
*/
void btGenSimbiconSPDController::CalculateControlForcesSimbiconSPD(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    // 1. calculate torso torque
    tVector3d torso_force = mExpJointPDControllers[0]
                                ->CalcControlForce(mTargetqCur, mTargetqdotCur)
                                .segment(0, 3);
    std::cout << "[simbicon_pd] torso force = " << torso_force.transpose()
              << std::endl;

    // 2. calculate J_stance and J_swing matrix
    tVectorXd q_cur = mModel->Getq(), qdot_cur = mModel->Getqdot();
    Joint *stance_joint =
              dynamic_cast<Joint *>(mModel->GetJointById(mStanceHipId)),
          *swing_joint =
              dynamic_cast<Joint *>(mModel->GetJointById(mSwingHipId));
    int stance_offset = stance_joint->GetOffset(),
        swing_offset = swing_joint->GetOffset();
    tMatrix3d J_stance = tMatrix3d::Zero(), J_swing = tMatrix3d::Zero();

    {
        auto GetJTorque = [](Joint *joint,
                             cRobotModelDynamics *model) -> tMatrix3d {
            auto parent_link = model->GetLinkById(joint->GetParentId());
            auto child_link = model->GetLinkById(joint->GetId());

            tMatrixXd J_diff = (child_link->GetJKw() - parent_link->GetJKw());
            tMatrix3d JTorque = J_diff.block(0, joint->GetOffset(), 3,
                                             joint->GetNumOfFreedom());
            return JTorque;
        };
        J_stance = GetJTorque(stance_joint, mModel);
        J_swing = GetJTorque(swing_joint, mModel);
    }

    // 3. calculate matrix E and vector f_vec
    int num_of_freedom = mModel->GetNumOfFreedom();
    tMatrixXd E = tMatrixXd::Identity(num_of_freedom, num_of_freedom);
    tVectorXd f_vec = tVectorXd::Zero(num_of_freedom);
    E.block(stance_offset, 0, 3, num_of_freedom).setZero();
    E.block(stance_offset, swing_offset, 3, 3) =
        -J_stance.transpose() * (J_swing.transpose()).inverse();
    f_vec.segment(stance_offset, 3) = -J_stance.transpose() * torso_force;

    // 4. calculate qddot_pred = (M + dt * Kd).inv() * (g + QG - C * qdot_cur)
    tVectorXd Kp = tVectorXd::Zero(num_of_freedom),
              Kd = tVectorXd::Zero(num_of_freedom);
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
        }
    }

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kp_mat = Kp.asDiagonal();
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kd_mat = Kd.asDiagonal();

    // std::cout << "E = \n" << E << std::endl;
    // std::cout << "f = " << f_vec.transpose() << std::endl;
    // 3.1 calculate g = E * Kp * (q_tar - q_cur - dt * qdot_cur) + E  * Kd * (qdot_tar - qdot_cur) + f
    tVectorXd g = E * Kp_mat * (mTargetqCur - q_cur - dt * qdot_cur) +
                  E * Kd_mat * (mTargetqdotCur - qdot_cur) + f_vec;

    tVectorXd qddot_pred =
        (mModel->GetMassMatrix() + dt * Kd_mat.toDenseMatrix()).inverse() *
        (g + mModel->CalcGenGravity(mWorld->GetGravity()) -
         mModel->GetCoriolisMatrix() * mModel->Getqdot());

    // 5. Q_tau' = E * Kp * (q_tar - q_cur - dt * qdot_cur) + E * Kd * (qdot_tar - qdot_cur - dt * qddot_pred) + f
    tVectorXd Q_tau =
        E * Kp_mat * (mTargetqCur - mModel->Getq() - dt * mModel->Getqdot()) +
        E * Kd_mat * (mTargetqdotCur - mModel->Getqdot() - dt * qddot_pred) +
        f_vec;

    std::cout << "[simbicon_pd] Q tau = " << Q_tau.transpose() << std::endl;
    tEigenArr<tVector3d> joint_torques;
    tVector3d root_force, root_torque;
    mModel->ConvertGenForceToCartesianForceTorque(Q_tau, joint_torques,
                                                  root_force, root_torque);

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
            double force_lim = mExpJointPDControllers[i]->GetForceLim();
            pd_force.mForce = btMathUtil::Expand(joint_torques[i - 1], 0);
            if (pd_force.mForce.norm() > force_lim)
                pd_force.mForce /= (pd_force.mForce.norm() / force_lim);
            // std::cout << "[simbicon_pd] joint " << i
            //           << " force = " << pd_force.mForce.transpose()
            //           << " , force lim = " << force_lim << std::endl;
            BTGEN_ASSERT(pd_force.mJoint->GetId() == i);
        }
        pd_forces.push_back(pd_force);
    }
    // VerifySimbiconSPD(dt, pd_forces);
}

/**
 * \brief           Check whether we should enable simbicon SPD
*/
bool btGenSimbiconSPDController::EnableSimbiconSPD() const
{
    BTGEN_ASSERT(mTargetUpdated && mJointIdUpdated);

    // if swing & stance id = -1, disable simbicon-SPD
    // else, enable simbicon-SPD
    return !(mSwingHipId == -1 && mStanceHipId == -1);
}

/**
 * \brief           Verify Simbicon SPD
 * make sure the torso's true applied torque is this value, which means \tau_torso = \tau_swing + \tau_stance
*/
void btGenSimbiconSPDController::VerifySimbiconSPD(
    double dt, tEigenArr<btGenPDForce> &pd_forces)
{
    BTGEN_ASSERT(mTargetUpdated && mJointIdUpdated);
    // 1. get swing and stance hip pd force
    tVector3d swing_force = tVector3d::Zero(), stance_force = tVector3d::Zero();
    BTGEN_ASSERT(pd_forces.size() == mModel->GetNumOfJoint());

    swing_force = pd_forces[mSwingHipId].mForce.segment(0, 3);
    stance_force = pd_forces[mStanceHipId].mForce.segment(0, 3);

    tVector3d torso_force = mExpJointPDControllers[0]
                                ->CalcControlForce(mTargetqCur, mTargetqdotCur)
                                .segment(0, 3);
    std::cout << "[verify] torso force = " << torso_force.transpose()
              << std::endl;
    std::cout << "[verify] -swing-stance force = "
              << (-swing_force - stance_force).transpose() << std::endl;
    tVector3d diff = -swing_force - stance_force - torso_force;

    double diff_norm = diff.norm();
    if (diff_norm > 1e-3)
    {
        std::cout << "[error] verify failed, torso diff = " << diff.transpose()
                  << std::endl;
        exit(0);
    }
    // BTGEN_ASSERT(diff_norm < 1e-3);c
}