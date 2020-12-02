#include "FixedRootJoint.h"
#include "tools.h"

/**
 * \brief           Fixed root joint, has only X rotation and YOZ translation, 3dof
*/
FixedRootJoint::FixedRootJoint(BaseObjectParams &param, int n_f) : Joint(param)
{
    BTGEN_ASSERT(n_f == 0 && "FixedRootJoint n_f is not zero!");

    parent = nullptr;
    joint_type = JointType::FIXED_NONE_JOINT;

    // y_translate, z_translate, x_revolute
    local_freedom = 0;
    freedoms.clear();
    dependent_dof_id.clear();

    // place the fixed root at (0, 1, 0)
    mFixedRootJointWorldTrans.setIdentity();
    mFixedRootJointWorldTrans(1, 3) = 1;
}

void FixedRootJoint::InitTerms()
{
    total_freedoms = GetNumOfFreedom();
    prev_freedoms = 0;
    InitGlobalToTotalFreedomMap();
    InitMatrix();
}

/**
 * \brief           Calcualte the rotation matrix for fixed root joint
 * 
 * set the 0 order and 1 order local info 
*/
void FixedRootJoint::GetRotations(tMatrix3d &m)
{
    // 1. set the value of translational freedoms of
    // local pos and local_pos_parent_joint
    {
        local_pos.setZero();
        local_pos_parent_joint.setZero();
        local_pos_parent_joint =
            this->mFixedRootJointWorldTrans.block(0, 3, 3, 1);
    }

    // 2. set the local transform matrix r_m: no local transform
    // 3. get the local rotaion matrix m
    m.noalias() = mFixedRootJointWorldTrans.block(0, 0, 3, 3);
}

void FixedRootJoint::Tell()
{
    std::cout << name << ":\nprev_freedom: " << total_freedoms << std::endl;
}

/**
 * \brief               Compute the d(local_transform)/dq
 * 
 * For FixedRootJoint, it has 0 freedoms, q = [y, z, theta_x]
 *                                               0  1    2
*/
void FixedRootJoint::ComputeLocalTransformFirstDerive() { mTq.clear(); }

/**
 * \brief               Compute the dd(local_transform)/dqiqj
*/
void FixedRootJoint::ComputeLocalTransformSecondDerive()
{
    // 1. give local pure second order derivation
    r_m_second_deriv.clear();

    // 2. give local multiplication pure second order derivation
    mTqq.clear();
}

/**
 * \brief               Compute mTqqq for fixed root joint
 * 
*/
void FixedRootJoint::ComputeLocalTransformThirdDerive()
{
    // 1. update the third deriv for 3 rotation freedoms
    r_m_third_deriv.clear();
}

/**
 * \brief           set the fixed root joint result
*/
void FixedRootJoint::SetFixedRootJointWorldTrans(const tMatrix &trans)
{
    mFixedRootJointWorldTrans = trans;
}

tMatrix FixedRootJoint::GetFixedRootJointWorldTrans() const
{
    return mFixedRootJointWorldTrans;
}