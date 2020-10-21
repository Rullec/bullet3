#include "LimitRootJoint.h"
#include "tools.h"

LimitRootJoint::LimitRootJoint(BaseObjectParams &param, int n_f) : Joint(param)
{
    if (n_f != 0)
    {
        std::cout << "[error] limit root joint nf " << n_f << " != 0 \n";
        exit(0);
    }
    parent = nullptr;
    joint_type = LIMIT_NONE_JOINT;
    // for (int i = TRANSLATE_Z; i <= TRANSLATE_Z; i++)
    // only add the z translate freedom
    {
        Freedom f;
        f.axis = tVector3d(0., 0., 0.);
        f.axis[TRANSLATE_Z] = 1.;
        f.lb = -INF;
        f.ub = INF;
        f.v = 0;
        f.id = n_f + 0;
        f.type = TRANSLATE;
        f.name = "Root";
        freedoms.push_back(f);
        dependent_dof_id.push_back(f.id);
    }
    local_freedom = static_cast<int>(freedoms.size());
}

void LimitRootJoint::InitTerms()
{
    total_freedoms = GetNumOfFreedom();
    prev_freedoms = 0;

    // the init matrix will allocate all matrices
    // the limit root joint has only 1 freedom: the z translate freedom
    InitMatrix();
}

/**
 * \brief           Get the rotation matrix3d "m"
 * 
 *  m = Rz * Ry * Rx, it will rotate a vector from local frame to parent frame
 * 
 *  In this limit root joint, it has no rotation freedom, so the m is always identity
*/
void LimitRootJoint::GetRotations(tMatrix3d &m)
{
    // set the z translate
    {
        local_pos[TRANSLATE_Z] = freedoms[0].v;
        local_pos_parent_joint[TRANSLATE_Z] = freedoms[0].v;
    }
    m.setIdentity();

    /* here r_m has only one element,
        r_m[0] means the r_m[TRANSLATE_Z]
        set the translate mat r_m
    */
    r_m[0].setIdentity();
    r_m[0](TRANSLATE_Z, 3) = local_pos[TRANSLATE_Z];

    // set the first deriv
    r_m_first_deriv[0].setZero();
    r_m_first_deriv[0](TRANSLATE_Z, 3) = 1;
}

void LimitRootJoint::Tell()
{
    std::cout << name << ":\nprev_freedom: " << total_freedoms << std::endl;
}

/**
 * limit root joint has only on z translation freedom
 * len(mTq) = 0
 * mTq = dTdq = dTdz
 * 
*/
void LimitRootJoint::ComputeTransformFirstDerive()
{
    mTq[0] = r_m_first_deriv[0];
}

/**
 * \brief           for limit root joint, the local transform T = T_z
 * d^2T/dz^2 = 0
*/
void LimitRootJoint::ComputeLocalTransformSecondDerive()
{
    mTqq[0][0].setZero();
}
