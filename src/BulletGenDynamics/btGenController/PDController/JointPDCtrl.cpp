#include "BulletGenDynamics/btGenController/PDController/JointPDCtrl.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/Link.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include <iostream>

btGenJointPDCtrl::btGenJointPDCtrl(cRobotModelDynamics *model, Joint *joint,
                                   double kp, double kd, double force_lim,
                                   bool use_world)
    : mUseWorldCoord(use_world)
{
    mModel = model;
    mJoint = joint;
    mKp = kp;
    mKd = kd;
    mForceLim = force_lim;
}

void btGenJointPDCtrl::SetKp(double Kp) { mKp = Kp; }
void btGenJointPDCtrl::SetKd(double Kd) { mKd = Kd; }
double btGenJointPDCtrl::GetKp(double Kp) const { return mKp; }
double btGenJointPDCtrl::GetKd(double Kd) const { return mKd; }
bool btGenJointPDCtrl::GetUseWorldCoord() const { return this->mUseWorldCoord; }

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
 *  * for spherical joint, it is 3x1 vector euler angles
 * for revolute joint, it is 1x1 vector angle
 * \param theta     If mUseWorldCoord is False, the theta should be whole dof q target
 *                  if mUseWorldCoord is True, the theta should be this joint's global orientation, expressed in quaternion
 * 
*/
tVector btGenJointPDCtrl::CalcControlForce(const tVectorXd &tar_q,
                                           const tVectorXd &tar_qdot) const
{

    int offset = mJoint->GetOffset();
    int size = this->GetCtrlDims();
    tVectorXd local_target_theta = tar_q.segment(offset, size);
    tVectorXd local_target_vel = tar_qdot.segment(offset, size);
    tVector force = tVector::Zero();
    switch (mJoint->GetJointType())
    {
    case JointType::NONE_JOINT:
        ControlForceNone(force, local_target_theta, local_target_vel);
        break;
    case JointType::SPHERICAL_JOINT:
        ControlForceSpherical(force, local_target_theta, local_target_vel);
        break;
    case JointType::REVOLUTE_JOINT:
        ControlForceRevolute(force, local_target_theta, local_target_vel);
        break;
    case JointType::FIXED_JOINT:
        ControlForceFixed(force, local_target_theta, local_target_vel);
    default:
        BTGEN_ASSERT(false);
        break;
    }

    if (force.norm() > mForceLim)
    {
        force = force.normalized() * mForceLim;
    }
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
 * in another word, only acount for the orientation target
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

    // 5. convert joint local force to global frame, rotate
    tVector3d global_force =
        mJoint->GetGlobalTransform().block(0, 0, 3, 3) * local_force;

    force.segment(0, 3) = global_force;
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
        mJoint->GetGlobalTransform().topLeftCorner<3, 3>() * local_force;
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

    // 3. global control force
    tVector3d global_force = mJoint->GetWorldOrientation() * local_force;
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
*/
void btGenJointPDCtrl::CalcLocalControlTarget(
    tVectorXd &local_control_target,
    const tQuaternion &target_global_quad) const
{
    // if this joint's target is in world coord, the local_control_target is the target global orientation of this joint, quaternio
    if (mUseWorldCoord == true)
    {

        /*
            calcualte the current local target 

            joint_global_rot = rest_rot * local_rot
            rest_rot = joint_global_rot * local_rot.inv()

            target_global_rot = joint->rest_rot * local_target 
            SO: local_target = target_global_rot * rest_rot.conj()            
        */
        tQuaternion cur_global_rot =
            btMathUtil::RotMat3dToQuaternion(mJoint->GetWorldOrientation());
        tQuaternion cur_local_rot =
            btMathUtil::RotMat3dToQuaternion(mJoint->GetRotations());
        tQuaternion cur_joint_rest_rot =
            cur_global_rot * cur_local_rot.conjugate();

        tQuaternion local_target =
            target_global_quad * cur_joint_rest_rot.conjugate();

        // convert this quaternion to the local control target

        tVectorXd control_target = tVectorXd(0);
        switch (mJoint->GetJointType())
        {
        case JointType::SPHERICAL_JOINT:
            control_target = btMathUtil::QuaternionToEulerAngles(
                                 local_target, btRotationOrder::bt_XYZ)
                                 .segment(0, 3);
            break;
        case JointType::REVOLUTE_JOINT:
        {
            // convert the local_target quaternion to the axis angle, then project it to the axis of the revolute freedom
            tVector3d local_target_aa =
                btMathUtil::QuaternionToAxisAngle(local_target).segment(0, 3);
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
                btMathUtil::QuaternionToEulerAngles(local_target,
                                                    btRotationOrder::bt_XYZ)
                    .segment(0, 3);
            break;
        case JointType::FIXED_JOINT:
            control_target = tVectorXd::Zero(0);
            break;
        default:
            std::cout << "joint type " << mJoint->GetJointType()
                      << " unsupported in joint pd ctrl\n";
            BTGEN_ASSERT(false);
            break;
        }
        local_control_target = control_target;
    }
}

void btGenJointPDCtrl::BuildTargetPose(tVectorXd &q)
{
    if (mUseWorldCoord == true)
    {
        // 1. set q (use the fastest way)
        tVectorXd q_old = mModel->Getq();
        mModel->Apply(q, false);
        tQuaternion tar_world_orient =
            btMathUtil::RotMat3dToQuaternion(mJoint->GetWorldOrientation());
        mModel->Apply(q_old, false);
        // 2. calculate the local target
        tVectorXd joint_target = q.segment(mJoint->GetOffset(), GetCtrlDims());
        CalcLocalControlTarget(joint_target, tar_world_orient);
        // 3. write the local target back
        q.segment(mJoint->GetOffset(), GetCtrlDims()) = joint_target;
    }
}
void btGenJointPDCtrl::BuildTargetVel(tVectorXd &qdot)
{
    // doesn't have any change
}