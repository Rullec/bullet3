#include "BipedalRootJoint.h"
#include "tools.h"

/**
 * \brief           Bipedal root joint, has only X rotation and YOZ translation, 3dof
*/
BipedalRootJoint::BipedalRootJoint(BaseObjectParams &param, int n_f)
    : Joint(param)
{
    BTGEN_ASSERT(n_f == 0 && "BipedalRootJoint n_f is not zero!");

    parent = nullptr;
    this->joint_type = JointType::BIPEDAL_NONE_JOINT;

    // y_translate, z_translate, x_revolute
    local_freedom = 0;
    for (auto &x : {TRANSLATE_Y, TRANSLATE_Z})
    {
        Freedom f;
        f.axis = tVector3d(0., 0., 0.);
        f.axis[x - TRANSLATE_X] = 1.;
        f.lb = -INF;
        f.ub = INF;
        f.v = 0;
        f.id = n_f + local_freedom;
        f.type = TRANSLATE;
        f.name = "Root";
        freedoms.push_back(f);
        dependent_dof_id.push_back(f.id);
        local_freedom++;
    }
    for (auto &x : {REVOLUTE_X})
    {
        Freedom f;
        f.axis = tVector3d(0., 0., 0.);
        f.axis[x - REVOLUTE_X] = 1.;
        f.lb = -M_PI;
        f.ub = M_PI;
        f.v = 0;
        f.id = n_f + local_freedom;
        f.type = REVOLUTE;
        f.name = "Root";
        freedoms.push_back(f);
        dependent_dof_id.push_back(f.id);
        local_freedom++;
    }
}

void BipedalRootJoint::InitTerms()
{
    total_freedoms = GetNumOfFreedom();
    prev_freedoms = 0;
    InitGlobalToTotalFreedomMap();
    InitMatrix();
}

/**
 * \brief           Calcualte the rotation matrix for bipedal root joint
 * r_m[0] : translate_y
 * r_m[1] : translate_z
 * r_m[2] : rotation mat of freedom x
 * 
 * set the 0 order and 1 order local info 
*/
void BipedalRootJoint::GetRotations(tMatrix3d &m)
{
    // 1. set the value of translational freedoms of
    // local pos and local_pos_parent_joint
    {
        local_pos.setZero();
        local_pos_parent_joint.setZero();
        // set freedom[0] value to translate y
        local_pos_parent_joint[1] = local_pos[1] = freedoms[0].v;

        // set freedom[1] value to translate z
        local_pos_parent_joint[2] = local_pos[2] = freedoms[1].v;
    }

    // 2. set the local transform matrix r_m
    // r_m size = local_freedoms = 3,
    // [trans_y, trans_z, rot_x]
    // by the value of x rotation freedom
    {
        // set r_m[0] for translational y freedom
        r_m[0].setIdentity();
        r_m[0](TRANSLATE_Y, 3) = local_pos[1];
        r_m_first_deriv[0].setZero();
        r_m_first_deriv[0](TRANSLATE_Y, 3) = 1;

        // set r_m[1] for translational z freedom
        r_m[1].setIdentity();
        r_m[1](TRANSLATE_Z, 3) = local_pos[2];
        r_m_first_deriv[1].setZero();
        r_m_first_deriv[1](TRANSLATE_Z, 3) = 1;

        // set r_m[2] for rotational x freedom
        xconventionTransform(r_m[2], freedoms[2].v);
        xconventionRotation_dx(r_m_first_deriv[2], freedoms[2].v);
    }

    // 3. get the local rotaion matrix m
    m.noalias() = r_m[2].topLeftCorner<3, 3>();
}

void BipedalRootJoint::Tell()
{
    std::cout << name << ":\nprev_freedom: " << total_freedoms << std::endl;
}

/**
 * \brief               Compute the d(local_transform)/dq
 * 
 * For BipedalRootJoint, it has 3 freedoms, q = [y, z, theta_x]
 *                                               0  1    2
 * so the final local transform is 
 *      T = Transz * Transy * Rx \in R^{4x4}
 
 * dT/dq_ty(id=0) = Transz * d(Transy)/dqy * Rx
 * dT/dq_tz(id=1) = d(Transz)/dqz * Transy * Rx
 * dT/dq_Rx(id=2) = Transz * Transy * d(Rx)/dqx
*/
void BipedalRootJoint::ComputeLocalTransformFirstDerive()
{
    mTq[0].noalias() = r_m[1] * r_m_first_deriv[0] * r_m[2];
    mTq[1].noalias() = r_m_first_deriv[1] * r_m[0] * r_m[2];
    mTq[2].noalias() = r_m[1] * r_m[0] * r_m_first_deriv[2];
}

/**
 * \brief               Compute the dd(local_transform)/dqiqj
 * 
 * T = Transz * Transy * Rx \in R^{4x4}
 * 
 * q0 = transy
 * q1 = transz
 * q2 = theta_x
 * 
 * mTqq[q0][q1] = d(Transz)dt_z * d(Transy)dty * Rx 
 * mTqq[q0][q2] = Transz * d(Transy)dty * d(Rx)dx 
 * mTqq[q1][q2] = d(Transz)/dtz * Transy * d(Rx)dx
 * 
 * mTqq[0][0] = 0
 * mTqq[1][1] = 0
 * mTqq[2][2] = Transz * Transy * dd(Rx)/dxx 
 * 
 * also, mTqq is symmetric
*/
void BipedalRootJoint::ComputeLocalTransformSecondDerive()
{
    // 1. give local pure second order derivation
    r_m_second_deriv[0].setZero();
    r_m_second_deriv[1].setZero();
    xconventionRotation_dxdx(r_m_second_deriv[2], this->freedoms[2].v);

    // 2. give local multiplication pure second order derivation
    mTqq[0][0].setZero();
    mTqq[1][1].setZero();
    mTqq[2][2].noalias() = r_m[1] * r_m[0] * r_m_second_deriv[2];

    // 3. others
    mTqq[1][0].noalias() = r_m_first_deriv[1] * r_m_first_deriv[0] * r_m[2];
    mTqq[2][0].noalias() = r_m[1] * r_m_first_deriv[0] * r_m_first_deriv[2];
    mTqq[2][1].noalias() = r_m_first_deriv[1] * r_m[0] * r_m_first_deriv[2];
}

/**
 * \brief               Compute mTqqq for bipedal root joint
 * 
 * T = Transz * Transy * Rx \in R^{4x4}
 * q = [translate_y, translate_z, rotate_x]
 *      0               1           2
 * 
*/
void BipedalRootJoint::ComputeLocalTransformThirdDerive()
{
    // 1. update the third deriv for 3 rotation freedoms
    r_m_third_deriv[0].setZero();
    r_m_third_deriv[1].setZero();
    xconventionRotation_dxdxdx(r_m_third_deriv[2], freedoms[2].v);

    // 2. begin to calculate mTqqq
    // T = Transz * Transy * Rx \in R^{4x4}
    // T = 1 * 0 * 2
    const int root_freedom_order[3] = {1, 0, 2};
    int *freedom_derivative_order = new int[local_freedom];
    tMatrix value;
    for (int i = local_freedom - 1; i >= 0; i--)
        for (int j = i; j >= 0; j--)
            for (int k = j; k >= 0; k--)
            {
                memset(freedom_derivative_order, 0,
                       sizeof(int) * local_freedom);
                freedom_derivative_order[i]++;
                freedom_derivative_order[j]++;
                freedom_derivative_order[k]++;
                value.setIdentity();
                for (int idx = 0; idx < local_freedom; idx++)
                {
                    switch (freedom_derivative_order[root_freedom_order[idx]])
                    {
                    case 0:
                        value = value * r_m[root_freedom_order[idx]];
                        break;
                    case 1:
                        value =
                            value * r_m_first_deriv[root_freedom_order[idx]];
                        break;
                    case 2:
                        value =
                            value * r_m_second_deriv[root_freedom_order[idx]];
                        break;
                    case 3:
                        value =
                            value * r_m_third_deriv[root_freedom_order[idx]];
                        break;

                    default:
                        std::cout << "[error] error derivative order "
                                  << freedom_derivative_order[idx] << std::endl;
                        exit(0);
                        break;
                    }
                }
                mTqqq[i][j][k] = value;
            }
    delete[] freedom_derivative_order;
}