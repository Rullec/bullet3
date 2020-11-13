#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include <iostream>

btGenJointPDCtrl::btGenJointPDCtrl(Joint *joint, double kp, double kd,
                                   double force_lim)
{
    mJoint = joint;
    mKp = kp;
    mKd = kd;
    mForceLim = force_lim;
}

void btGenJointPDCtrl::SetKp(double Kp) { mKp = Kp; }
void btGenJointPDCtrl::SetKd(double Kd) { mKd = Kd; }
double btGenJointPDCtrl::GetKp(double Kp) const { return mKp; }
double btGenJointPDCtrl::GetKd(double Kd) const { return mKd; }

/**
 * \brief           Set the target (euler) angle in q (generalized coordinate)
 * for spherical joint, it is 3x1 vector euler angles
 * for revolute joint, it is 1x1 vector angle
*/
void btGenJointPDCtrl::SetTargetTheta(const tVectorXd &theta)
{
    CheckCtrlDims(theta, "SetTargetTheta");
    mTargetTheta = theta;
}

void btGenJointPDCtrl::SetTargetVel(const tVectorXd &vel)
{
    CheckCtrlDims(vel, "SetTargetVel");
    mTargetVel = vel;
}

tVectorXd btGenJointPDCtrl::GetTargetTheta() const { return mTargetTheta; }
tVectorXd btGenJointPDCtrl::GetTargetVel() const { return mTargetVel; }
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
*/
tVector btGenJointPDCtrl::CalcControlForce() const
{
    tVector force = tVector::Zero();
    switch (mJoint->GetJointType())
    {
    case JointType::NONE_JOINT:

        break;

    default:
        break;
    }
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
 * \brief           Check the dims of var is equal to control dims (aka joint dof)
*/
void btGenJointPDCtrl::CheckCtrlDims(const tVectorXd &var,
                                     std::string prefix) const
{
    if (var.size() != GetCtrlDims())
    {
        printf("[error] CheckCtrlDims failed for %s, %d != %d\n",
               prefix.c_str(), var.size(), GetCtrlDims());
        exit(0);
    }
}

/**
 * \brief           control the force for none joint
 * 
 * Usually there should be not actuated control force on root joint,
 * because it's not controllable. But in simbicon, we needs to calculate
 * the virtual torque on the torso in world frame.
 * 
 * this function will calculate the virtual torque on none joint the same as a spherical joint 
 * in another word, only acount for the orientation target
*/
void btGenJointPDCtrl::ControlForceNone(tVector &force) const
{
    force.setZero();
    // 1. get the target orientation quaternion
    tVector3d target_orient_xyz = mTargetTheta.segment(3, 3);
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
    tVector3d target_joint_vel = mTargetVel.segment(3, 3);
    tVector3d cur_joint_vel = this->mJoint->GetJointLocalVel().segment(3, 3);
    // transform joint vel to omega
    tVector3d target_omega = btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
        target_joint_vel, btRotationOrder::bt_XYZ);
    tVector3d cur_omega = btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
        cur_joint_vel, btRotationOrder::bt_XYZ);
    tVector3d omega_diff = target_omega - cur_omega;
    tVector3d local_force = mKp * orient_diff + mKd * omega_diff;

    // 5. convert joint local force to global frame, rotate
    tVector3d global_force =
        mJoint->GetGlobalTransform().block(0, 0, 3, 3) * local_force;

    force.segment(0, 3) = global_force;
}

/**
 * \brief           Calculate the global PD control force for revolute joint
 * \param force     global PD control force
*/
void btGenJointPDCtrl::ControlForceRevolute(tVector &force) const
{
    force.setZero();
    // 1. get current and target value
    auto single_freedom = mJoint->GetFreedoms(0);
    double target_orient = this->mTargetTheta[0],
           cur_orient = single_freedom->v;
    double target_vel = mTargetVel[0], cur_vel = single_freedom->vdot;

    // 2. get diff and final force
    double orient_diff = target_orient - cur_orient,
           vel_diff = target_vel - cur_vel;
    tVector3d local_force =
        single_freedom->axis * (mKp * orient_diff + this->mKd * vel_diff);
    force.segment(0, 3) =
        mJoint->GetGlobalTransform().topLeftCorner<3, 3>() * local_force;
}

/**
 * \brief           Calculate the global PD
 * \param force     global PD control force
*/
void btGenJointPDCtrl::ControlForceSpherical(tVector &force) const
{
    // 1. get the target and current quantities
    tVector3d target_orient_xyz = mTargetTheta.segment(0, 3);
    tQuaternion target_orient = btMathUtil::EulerAnglesToQuaternion(
        btMathUtil::Expand(target_orient_xyz, 0), btRotationOrder::bt_XYZ);
    tQuaternion cur_orient =
        btMathUtil::RotMatToQuaternion(this->mJoint->GetGlobalTransform());

    // joint vel: xdot, ydot, zdot of euler angles
    // joint omega: angular velocity
    tVector3d target_joint_vel = mTargetVel.segment(0, 3);
    tVector3d cur_joint_vel = mJoint->GetJointLocalVel().segment(0, 3);
    tVector3d target_omega = btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
        target_joint_vel, btRotationOrder::bt_XYZ);
    tVector3d cur_omega = btMathUtil::ConvertEulerAngleVelToAxisAngleVel(
        cur_joint_vel, btRotationOrder::bt_XYZ);

    // 2. calc the diff
    tVector3d omega_diff = target_omega - cur_omega;
    tVector3d orient_diff = btMathUtil::QuaternionToAxisAngle(
                                target_orient * cur_orient.conjugate())
                                .segment(0, 3);
    tVector3d local_force = mKp * orient_diff + this->mKd * omega_diff;

    // 3. global control force
    tVector3d global_force = mJoint->GetWorldOrientation() * local_force;
    force.segment(0, 3);
    force.segment(0, 3) = global_force;
}

/**
 * \brief           No control force availiable for fixed joint
*/
void btGenJointPDCtrl::ControlForceFixed(tVector &force) const
{
    force.setZero();
}