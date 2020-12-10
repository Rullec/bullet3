#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include <iostream>

btGenJointPDCtrl::btGenJointPDCtrl(cRobotModelDynamics *model, Joint *joint,
                                   double kp, double kd, const tVector3d &scale,
                                   double force_lim, bool use_world)
    : mUseWorldCoord(use_world), mScale(scale)
{
    mModel = model;
    mJoint = joint;
    mKp = kp;
    mKd = kd;
    mForceLim = force_lim;

    if (mUseWorldCoord)
        printf("[log] joint %d %s use world PD coord\n", joint->GetId(),
               joint->GetName().c_str());
}

void btGenJointPDCtrl::SetKp(double Kp) { mKp = Kp; }
void btGenJointPDCtrl::SetKd(double Kd) { mKd = Kd; }
double btGenJointPDCtrl::GetKp() const { return mKp; }
double btGenJointPDCtrl::GetKd() const { return mKd; }
bool btGenJointPDCtrl::GetUseWorldCoord() const { return this->mUseWorldCoord; }

void btGenJointPDCtrl::SetUseWorldCoord(bool val) { mUseWorldCoord = val; }
Joint *btGenJointPDCtrl::GetJoint() const { return mJoint; }

/**
 * \brief           Calculate the control force at this joint
 * for revolute, spherical, none joint, the control force = control torque
 * 
 * the calculated torque is expressed in world frame,
 * if mUseWorldCoord = false, the control torque will drive the local angle of this joint to the target. in this case, the target should also be a local angle
 * if mUseWorldCoord = true, the control torque will drive the global orientation of this JOINT to the target
 *  Note that, it is the global orientation OF THE JOINT, not the child LINK.
 * 
 * for spherical joint, it is 3x1 vector euler angles
 * for revolute joint, it is 1x1 vector angle
 * \param theta     If mUseWorldCoord is False, the theta should be whole dof q target
 *                  if mUseWorldCoord is True, the theta should be this joint's global orientation, expressed in quaternion
 * 
*/
tVector
btGenJointPDCtrl::CalcControlForce(const tVectorXd &control_tar_q,
                                   const tVectorXd &control_tar_qdot) const
{
    int offset = mJoint->GetOffset();
    int size = this->GetCtrlDims();
    tVectorXd local_target_theta = control_tar_q.segment(offset, size);
    tVectorXd local_target_vel = control_tar_qdot.segment(offset, size);
    tVector force = tVector::Zero();
    switch (mJoint->GetJointType())
    {
    case JointType::NONE_JOINT:
        ControlForceNone(force, local_target_theta, local_target_vel);
        break;
    case JointType::BIPEDAL_NONE_JOINT:
        ControlForceBipedalNone(force, local_target_theta, local_target_vel);
        break;
    case JointType::SPHERICAL_JOINT:
        ControlForceSpherical(force, local_target_theta, local_target_vel);
        break;
    case JointType::REVOLUTE_JOINT:
        ControlForceRevolute(force, local_target_theta, local_target_vel);
        break;
    case JointType::FIXED_JOINT:
    case JointType::FIXED_NONE_JOINT:
        ControlForceFixed(force, local_target_theta, local_target_vel);
        break;
    case JointType::LIMIT_NONE_JOINT:
        ControlForceLimitNone(force, local_target_theta, local_target_vel);
        break;
    case JointType::UNIVERSAL_JOINT:
        ControlForceUniversal(force, local_target_theta, local_target_vel);
        break;
    default:
        BTGEN_ASSERT(false);
        break;
    }
    if (force.norm() > mForceLim)
    {
        force = force.normalized() * mForceLim;
    }
    // std::cout << "[pd] joint " << mJoint->GetId() << " force = " << force.transpose()
    //           << " lim = " << mForceLim << std::endl;

    // DebugVerifyCtrlForceKp(local_target_theta, force);
    // DebugVerifyCtrlForceKd(local_target_vel, force);
    return force;
}

/**
 * \brief           Get the control dimensions, equal to the joint dof
 * for revolute, 1
 * for spherical, 3, euler angles [x, y, z]
 * for fixed, 0
 * for none, 6, [x,y,z, ax,ay,az]
*/
int btGenJointPDCtrl::GetCtrlDims() const { return mJoint->GetNumOfFreedom(); }

/**
 * \brief           control the force for none joint
 * 
 * Usually there should be not actuated control force on root joint,
 * because it's not controllable. But in simbicon, we needs to calculate
 * the virtual torque on the torso in world frame.
 * 
 * this function will calculate the virtual torque on none joint the same as a spherical joint 
 * in another word, only account for the orientation target
*/
void btGenJointPDCtrl::ControlForceNone(tVector &force,
                                        const tVectorXd &local_target_theta,
                                        const tVectorXd &local_target_vel) const
{
    force.setZero();
    // 1. get the target orientation quaternion
    tVector3d target_orient_xyz = local_target_theta.segment(3, 3);
    tQuaternion tar_orient = btMathUtil::EulerAnglesToQuaternion(
        btMathUtil::Expand(target_orient_xyz, 0), btRotationOrder::bt_XYZ);

    // 2. get the current orientation quaternion
    tMatrix cur_orient_mat = tMatrix::Identity();
    cur_orient_mat.block(0, 0, 3, 3) =
        mJoint->GetGlobalTransform().block(0, 0, 3, 3);
    tQuaternion cur_orient = btMathUtil::RotMatToQuaternion(cur_orient_mat);

    // 3. calculate the diff axis angle, target_orien = diff *  cur_orient, diff = target * cur.inv()
    tQuaternion orient_diff_qua = tar_orient * cur_orient.conjugate();
    tVector3d orient_diff = btMathUtil::QuaternionToAxisAngle(orient_diff_qua)
                                .segment(0, 3); // axis angle orientation diff

    /*
        4. calc the Kd * (target_omega - cur_omega)

        omega = Trans(joint_vel), here we need to transform the joint vel to local joint omega
    */
    tVectorXd target_joint_vel = local_target_vel;
    tVectorXd cur_joint_vel = mJoint->GetJointLocalVel();
    // transform joint vel to omega
    // tVector3d target_omega =
    //     btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
    //         btMathUtil::Expand(target_joint_vel, 0), btRotationOrder::bt_XYZ)
    //         .segment(0, 3);
    // tVector3d cur_omega =
    //     btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
    //         btMathUtil::Expand(cur_joint_vel, 0), btRotationOrder::bt_XYZ)
    //         .segment(0, 3);
    // std::cout << "root local jkw = \n" << mJoint->GetLocalJkw() << std::endl;
    tVector3d target_omega = mJoint->GetLocalJkw() * local_target_vel;
    tVector3d cur_omega = mJoint->GetLocalJkw() * cur_joint_vel;
    // std::cout << "root local jkw = \n" << mJoint->GetLocalJkw() << std::endl;
    // exit(0);
    tVector3d omega_diff = target_omega - cur_omega;
    tVector3d local_force = mKp * orient_diff + mKd * omega_diff;
    // std::cout << "[before_scale] root local force = " << local_force.transpose()
    //           << std::endl;
    local_force[0] *= mScale[0];
    local_force[1] *= mScale[1];
    local_force[2] *= mScale[2];
    // std::cout << "[after_scale] root local force = " << local_force.transpose()
    //           << std::endl;
    // 5. convert joint local force to global frame, rotate
    tVector3d global_force =
        mJoint->GetWorldOrientation() *
        mJoint->GetLocalTransform().block(0, 0, 3, 3).transpose() * local_force;
    force.segment(0, 3) = global_force;
    if (force.hasNaN())
    {
        std::cout << "[pd] none force has Nan, local force = "
                  << local_force.transpose()
                  << " global force = " << global_force.transpose()
                  << std::endl;
        std::cout << "orient diff = " << orient_diff.transpose() << std::endl;
        std::cout << "orient diff qua = "
                  << orient_diff_qua.coeffs().transpose() << std::endl;
        //    * .conjugate()
        std::cout << "tar_orient qua = " << tar_orient.coeffs().transpose()
                  << std::endl;
        std::cout << "tar_orient euler = " << target_orient_xyz.transpose()
                  << std::endl;
        // std::cout << "cur_orient conj qua = "
        //           << cur_orient.conjugate().coeffs().transpose() << std::endl;
        // std::cout << "vel diff = " << omega_diff.transpose() << std::endl;
        exit(0);
    }
}

/**
 * \brief           Calculate the global PD control force for revolute joint
 * \param force     global PD control force
*/
void btGenJointPDCtrl::ControlForceRevolute(
    tVector &force, const tVectorXd &local_target_theta,
    const tVectorXd &local_target_vel) const
{
    force.setZero();
    // 1. get current and target value
    auto single_freedom = mJoint->GetFreedoms(0);
    double target_orient = local_target_theta[0],
           cur_orient = single_freedom->v;
    double target_vel = local_target_vel[0], cur_vel = single_freedom->vdot;

    // 2. get diff and final force
    double orient_diff = target_orient - cur_orient,
           vel_diff = target_vel - cur_vel;
    tVector3d local_force =
        single_freedom->axis * (mKp * orient_diff + this->mKd * vel_diff);
    force.segment(0, 3) =
        mJoint->GetGlobalTransform().topLeftCorner<3, 3>() *
        mJoint->GetLocalTransform().block(0, 0, 3, 3).transpose() * local_force;
}

/**
 * \brief           Calculate the global PD
 * \param force     global PD control force
*/
void btGenJointPDCtrl::ControlForceSpherical(
    tVector &force, const tVectorXd &local_target_theta,
    const tVectorXd &local_target_vel) const
{
    // 1. get the target and current quantities
    tVector3d target_orient_xyz = local_target_theta.segment(0, 3);
    tQuaternion target_orient = btMathUtil::EulerAnglesToQuaternion(
        btMathUtil::Expand(target_orient_xyz, 0), btRotationOrder::bt_XYZ);
    tQuaternion cur_orient =
        btMathUtil::RotMatToQuaternion(this->mJoint->GetLocalTransform());

    // joint vel: xdot, ydot, zdot of euler angles
    // joint omega: angular velocity
    tVector3d target_joint_vel = local_target_vel.segment(0, 3);
    tVector3d cur_joint_vel = mJoint->GetJointLocalVel().segment(0, 3);
    // tVector3d target_omega =
    //     btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
    //         btMathUtil::Expand(target_joint_vel, 0), btRotationOrder::bt_XYZ)
    //         .segment(0, 3);
    // tVector3d cur_omega =
    //     btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
    //         btMathUtil::Expand(cur_joint_vel, 0), btRotationOrder::bt_XYZ)
    //         .segment(0, 3);
    tVector3d target_omega = mJoint->GetLocalJkw() * target_joint_vel;
    tVector3d cur_omega = mJoint->GetLocalJkw() * cur_joint_vel;

    // 2. calc the diff
    tVector3d omega_diff = target_omega - cur_omega;
    tVector3d orient_diff = btMathUtil::QuaternionToAxisAngle(
                                target_orient * cur_orient.conjugate())
                                .segment(0, 3);
    // std::cout << "target omega = " << target_omega.transpose() << std::endl;
    // std::cout << "cur omega = " << cur_omega.transpose() << std::endl;
    // std::cout << "omega diff = " << omega_diff.transpose() << std::endl;
    tVector3d local_force = mKp * orient_diff + mKd * omega_diff;

    // std::cout << "[before_scale] sph local force = " << local_force.transpose()
    //           << std::endl;
    local_force[0] *= mScale[0];
    local_force[1] *= mScale[1];
    local_force[2] *= mScale[2];
    // std::cout << "[after_scale] sph local force = " << local_force.transpose()
    //           << std::endl;
    // 3. global control force
    tVector3d global_force =
        mJoint->GetWorldOrientation() *
        mJoint->GetLocalTransform().block(0, 0, 3, 3).transpose() * local_force;

    force.segment(0, 3) = global_force;
}

/**
 * \brief           No control force availiable for fixed joint
*/
void btGenJointPDCtrl::ControlForceFixed(
    tVector &force, const tVectorXd &local_target_theta,
    const tVectorXd &local_target_vel) const
{
    force.setZero();
}

/**
 * \brief           Get the local control target  (PD Target theta in joint's local frame)
 *  This function can only be calculated when mUseWorldCoord = true
*/
tVectorXd btGenJointPDCtrl::CalcLocalControlTargetByWorldTarget(
    const tQuaternion &target_global_quad) const
{
    // if this joint's target is in world coord, the local_control_target is the target global orientation of this joint, quaternio
    BTGEN_ASSERT(mUseWorldCoord == true);
    /*
            calcualte the current local target 

            joint_global_rot = rest_rot * local_rot
            rest_rot = joint_global_rot * local_rot.inv()

            target_global_rot = joint->rest_rot * local_target 
            SO: local_target = target_global_rot * rest_rot.conj()            
        */
    tMatrix3d cur_global_rot = mJoint->GetWorldOrientation();

    tMatrix3d cur_local_rot = mJoint->GetRotations();
    tMatrix3d cur_joint_rest_rot = cur_global_rot * cur_local_rot.transpose();

    tMatrix local_target = btMathUtil::ExpandMat(
        cur_joint_rest_rot.transpose() *
            btMathUtil::RotMat(target_global_quad).block(0, 0, 3, 3),
        0);

    // convert this quaternion to the local control target

    tVectorXd control_target = tVectorXd(0);
    switch (mJoint->GetJointType())
    {
    case JointType::SPHERICAL_JOINT:
        control_target = btMathUtil::RotmatToEulerAngle(local_target,
                                                        btRotationOrder::bt_XYZ)
                             .segment(0, 3);

        // {
        //     tMatrix new_local_rot = btMathUtil::EulerAnglesToRotMat(
        //         btMathUtil::Expand(control_target, 0), btRotationOrder::bt_XYZ);
        //     {
        //         tMatrix local_diff = local_target - new_local_rot;
        //         double norm = local_diff.norm();
        //         std::cout << "local diff = " << norm << std::endl;
        //         BTGEN_ASSERT(norm < 1e-6);
        //     }

        //     tMatrix diff =
        //         btMathUtil::ExpandMat(cur_joint_rest_rot, 0) * new_local_rot -
        //         btMathUtil::RotMat(target_global_quad);
        //     double norm = diff.norm();
        //     std::cout << "[pd] CalcLocalTarget: spherical diff norm = " << norm
        //               << std::endl;
        //     std::cout << "diff = \n" << diff << std::endl;
        //     BTGEN_ASSERT(norm < 1e-6);
        // }
        break;
    case JointType::REVOLUTE_JOINT:
    {
        // convert the local_target quaternion to the axis angle, then project it to the axis of the revolute freedom
        tVector3d local_target_aa =
            btMathUtil::RotmatToAxisAngle(local_target).segment(0, 3);
        tVector3d axis = mJoint->GetFreedoms(0)->axis;
        BTGEN_ASSERT(std::fabs(axis.norm() - 1) < 1e-10);
        double target_angle = local_target_aa.dot(axis);
        // printf("revolute target angle PD is %.3f\n", target_angle);
        // std::cout << "axis = " << axis.transpose() << std::endl;
        // std::cout << "local target aa = " << local_target_aa.transpose()
        //           << std::endl;
        // std::cout << "joint current value = " << mJoint->GetFreedoms(0)->v
        //           << std::endl;
        control_target = tVectorXd::Ones(1) * target_angle;
    }
    break;
    case JointType::NONE_JOINT:
        // for none joint, the same as spherical joint
        control_target = tVectorXd::Zero(6);
        control_target.segment(3, 3) =
            btMathUtil::RotmatToAxisAngle(local_target).segment(0, 3);
        {
            if (control_target.hasNaN())
            {
                std::cout << "control target none has Nan = "
                          << control_target.transpose() << std::endl;
                std::cout << "local target = \n" << local_target << std::endl;
                std::cout << "local target qua = \n"
                          << btMathUtil::RotMatToQuaternion(local_target)
                                 .coeffs()
                                 .transpose()
                          << std::endl;

                exit(1);
            }
        }
        break;
    case JointType::FIXED_JOINT:
        control_target = tVectorXd::Zero(0);
        break;
    case JointType::BIPEDAL_NONE_JOINT:
    {
        control_target = tVectorXd::Zero(3);
        tVector aa = btMathUtil::RotmatToEulerAngle(local_target,
                                                    btRotationOrder::bt_XYZ);
        BTGEN_ASSERT(std::fabs(aa[1]) < 1e-10);
        BTGEN_ASSERT(std::fabs(aa[2]) < 1e-10);
        control_target[2] = aa[0];
    }
    break;
    default:
        std::cout << "joint type " << mJoint->GetJointType()
                  << " unsupported in joint pd ctrl\n";
        BTGEN_ASSERT(false);
        break;
    }
    if (control_target.hasNaN() == true)
    {
        std::cout << "control target = " << control_target.transpose()
                  << std::endl;
        exit(1);
    }
    return control_target;
}

/**
 * \brief           given a full size target q, calculate the local control target of this joint
 * if the mUseWorldCoord = True, this function will convert
 * the given target to another local given target, but can 
 * make the target joint's world orientation
 * is the same as desired
 * 
 * \param full_q_target     calculate the 
 * 
 * 
*/
tVectorXd
btGenJointPDCtrl::CalcJointTargetPose(const tVectorXd &full_q_target) const
{
    int offset = mJoint->GetOffset(), size = mJoint->GetNumOfFreedom();
    tVectorXd joint_local_target = tVectorXd::Zero(size);
    if (mUseWorldCoord == true)
    {
        // in world coord
        // 1. set q (use the fastest way)
        tVectorXd q_old = mModel->Getq();
        mModel->Apply(full_q_target, false);
        tQuaternion tar_world_orient =
            btMathUtil::RotMat3dToQuaternion(mJoint->GetWorldOrientation());

        mModel->Apply(q_old, false);
        // 2. calculate the local target

        joint_local_target =
            CalcLocalControlTargetByWorldTarget(tar_world_orient);
    }
    else
    {
        // doesn't in world coord
        joint_local_target = full_q_target.segment(offset, size);
    }
    return joint_local_target;
}
tVectorXd btGenJointPDCtrl::CalcJointTargetVel(const tVectorXd &qdot) const
{
    int offset = mJoint->GetOffset(), size = mJoint->GetNumOfFreedom();
    return qdot.segment(offset, size);
}

double btGenJointPDCtrl::GetForceLim() const { return this->mForceLim; }

/**
 * \brief           control the PD force for bipedal none joint
 * 
 * Usually the rootj oint is underactuated; but sometimes (such as in SIMBICON) 
 * we still needs to compute the virtual PD control on that
 * 
 * This function calculates the control TORQUE (not cartesian force, but only torque) for bipedal root
 * We assume that the only one rotational freedom in bipedal root is along with X-axis
 * so the result torque looks like: [N, 0, 0] parallel with X-axis
*/
void btGenJointPDCtrl::ControlForceBipedalNone(
    tVector &force, const tVectorXd &local_target_theta,
    const tVectorXd &local_target_vel) const
{
    BTGEN_ASSERT(mJoint->GetJointType() == JointType::BIPEDAL_NONE_JOINT);
    BTGEN_ASSERT(mJoint->GetNumOfFreedom() == 3);
    BTGEN_ASSERT(local_target_theta.size() == 3);
    auto rotate_freedom = mJoint->GetFreedoms(2);
    BTGEN_ASSERT(rotate_freedom->type == REVOLUTE);
    BTGEN_ASSERT((rotate_freedom->axis - tVector3d(1, 0, 0)).norm() < 1e-10);

    double cur_theta = rotate_freedom->v, cur_vel = rotate_freedom->vdot;
    double target_theta = local_target_theta[2],
           target_vel = local_target_vel[2];

    force.setZero();
    force[0] = this->mKp * (target_theta - cur_theta) +
               this->mKd * (target_vel - cur_vel);
}

/**
 * \brief           Verify the control force should have the same global orientation w.r.t the rotation difference expressed in aa
 * when Kd = 0
*/
void btGenJointPDCtrl::DebugVerifyCtrlForceKp(const tVectorXd &local_target,
                                              const tVector &calc_torque)
{
    BTGEN_ASSERT(std::fabs(mKd) < 1e-10);
    // 1. get the current joint global orient
    BTGEN_ASSERT(local_target.size() == mJoint->GetNumOfFreedom());
    mModel->PushState("debug_verify_pd");
    tMatrix3d cur_orient = mJoint->GetWorldOrientation();
    tVectorXd q = mModel->Getq();
    q.segment(this->mJoint->GetOffset(), mJoint->GetNumOfFreedom()) =
        local_target;
    mModel->SetqAndqdot(q, mModel->Getqdot());

    // 2. get the ideal joint global orient
    tMatrix3d ideal_orient = mJoint->GetWorldOrientation();

    // 3. calc the diff, and the diff direction
    // ideal = diff * current
    // diff = ideal * cur.inv
    tVector ideal_dir = btMathUtil::QuaternionToAxisAngle(
                            btMathUtil::RotMat3dToQuaternion(
                                ideal_orient * cur_orient.transpose()))
                            .normalized();
    // 4. verify the direction should be the same
    tVector truth_dir = calc_torque.normalized();
    tVector error;
    if ((ideal_dir - truth_dir).norm() < (ideal_dir + truth_dir).norm())
        error = (ideal_dir - truth_dir);
    else
        error = (ideal_dir + truth_dir);
    if (error.norm() > 1e-5)
    {
        std::cout << "[error] joint " << mJoint->GetId()
                  << " verify ctrl force failed, error diff = "
                  << error.transpose() << std::endl;
        std::cout << "\tideal dir = " << ideal_dir.transpose() << std::endl;
        std::cout << "\ttrue dir = " << truth_dir.transpose() << std::endl;
        std::cout << "\tcalc_torque = " << calc_torque.transpose() << std::endl;
        exit(0);
    }
    std::cout << "[pd] debug is enabled! kd is set to zero, verified succ\n";
    mModel->PopState("debug_verify_pd");
}

/**
 * \brief           Verify the control force should have the same global orientation w.r.t the rotation difference expressed in aa
 * when Kp = 0
*/
void btGenJointPDCtrl::DebugVerifyCtrlForceKd(
    const tVectorXd &local_target_theta_dot, const tVector &torque)
{
    BTGEN_ASSERT(std::fabs(this->mKp) < 1e-10);
    mModel->PushState("debug_verify_pd_kd");

    // 1. get current world omega of this joint (when other qdot is set to zero)
    tVectorXd qdot = mModel->Getqdot();
    int offset = mJoint->GetOffset();
    int dof = mJoint->GetNumOfFreedom();
    tVectorXd tmp = qdot.segment(offset, dof);
    qdot.setZero();
    qdot.segment(offset, dof) = tmp;
    mModel->SetqAndqdot(mModel->Getq(), qdot);

    tVector3d cur_omega = mJoint->GetJKw() * qdot;

    // 2. get target world omega
    qdot.segment(offset, dof) = local_target_theta_dot;
    tVector3d ideal_omega = mJoint->GetJKw() * qdot;

    // 3. ideal direction
    tVector3d ideal_torque_dir = (ideal_omega - cur_omega).normalized(),
              truth_torque_dir = torque.segment(0, 3).normalized();

    // 4. calculate error when the input torque is nonzero
    tVector3d error = ideal_torque_dir - truth_torque_dir;
    if (error.norm() > 1e-5 && torque.norm() > 1e-5)
    {
        std::cout << "[error] joint " << mJoint->GetId()
                  << " Kd PD verification failed, error = " << error.transpose()
                  << std::endl;
        std::cout << "cur omega = " << cur_omega.transpose() << std::endl;
        std::cout << "ideal omega = " << ideal_omega.transpose() << std::endl;

        std::cout << "ideal torque dir = " << ideal_torque_dir.transpose()
                  << std::endl;
        std::cout << "truth torque dir = " << truth_torque_dir.transpose()
                  << std::endl;
        exit(0);
    }
    mModel->PopState("debug_verify_pd_kd");
}

/**
 * \brief           calculate PD force for limit none
*/
void btGenJointPDCtrl::ControlForceLimitNone(
    tVector &force, const tVectorXd &local_target_theta,
    const tVectorXd &local_target_vel) const
{
    auto joint = GetJoint();
    BTGEN_ASSERT(JointType::LIMIT_NONE_JOINT == joint->GetJointType());

    // limit none joint, only has x translation force
    force.setZero();
    double cur_theta = mModel->Getq()[joint->GetOffset()],
           cur_vel = mModel->Getqdot()[joint->GetOffset()];
    force[0] = mKp * (local_target_theta[0] - cur_theta) +
               mKd * (local_target_vel[0] - cur_vel);
}

/**
 * \brief           calc force for force
*/
void btGenJointPDCtrl::ControlForceUniversal(
    tVector &force, const tVectorXd &local_target_theta,
    const tVectorXd &local_target_vel) const
{
    tVectorXd local_cur_vel = mJoint->GetJointLocalVel();
    tVectorXd local_cur_theta = mJoint->GetJointLocalTheta();
    tMatrixXd axis_mat = tMatrixXd::Zero(3, 2);
    axis_mat.col(0) = mJoint->GetFreedoms(0)->axis;
    axis_mat.col(1) = mJoint->GetFreedoms(1)->axis;

    tVector3d local_force =
        axis_mat * (mKp * (local_target_theta - local_cur_theta) +
                    mKd * (local_target_vel - local_cur_vel));
    force.segment(0, 3) = mJoint->GetWorldOrientation() * local_force;
}