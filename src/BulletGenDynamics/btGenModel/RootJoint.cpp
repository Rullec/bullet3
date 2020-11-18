#include "RootJoint.h"
#include "tools.h"

RootJoint::RootJoint(BaseObjectParams &param, int n_f) : Joint(param)
{
    parent = nullptr;
    joint_type = NONE_JOINT;
    for (int i = TRANSLATE_X; i < REVOLUTE_X; i++)
    {
        Freedom f;
        f.axis = tVector3d(0., 0., 0.);
        f.axis[i] = 1.;
        f.lb = -INF;
        f.ub = INF;
        // if (i == TRANSLATE_Y) f.ub = 0.37;
        f.v = 0;
        f.id = i + n_f;
        f.type = TRANSLATE;
        f.name = "Root";
        freedoms.push_back(f);
        dependent_dof_id.push_back(f.id);
    }
    for (int i = REVOLUTE_X; i < TOTAL_ROOT_FREEDOM; i++)
    {
        Freedom f;
        f.axis = tVector3d(0., 0., 0.);
        f.axis[i - REVOLUTE_X] = 1;
        f.lb = -3.14;
        f.ub = 3.14;

        // if (i == REVOLUTE_X) {
        //	f.lb = -0.5;
        //	f.ub = 0.5;
        //}

        f.v = 0;
        f.id = i + n_f;
        f.name = "Root";
        freedoms.push_back(f);
        dependent_dof_id.push_back(f.id);
    }
    local_freedom = static_cast<int>(freedoms.size());
}

void RootJoint::InitTerms()
{
    total_freedoms = GetNumOfFreedom();
    prev_freedoms = 0;
    InitGlobalToTotalFreedomMap();
    InitMatrix();
}

/**
 * \brief           Calculate the rotation matrices for root joint
 * 
 * the root joint has 6 freedoms, so the "r_m" member has 6 elements
 * 
 * r_m[0, 1, 2] = translate mat of freedom "x, y, z"
 * r_m[3, 4, 5] = rotation mat of freedom "x, y, z"
*/
void RootJoint::GetRotations(tMatrix3d &m)
{
    for (int i = TRANSLATE_X; i < REVOLUTE_X; i++)
    {
        local_pos[i] = freedoms[i].v;
        local_pos_parent_joint[i] = freedoms[i].v;
    }
    m.setIdentity();

    // calculate theta_x, theta_y, theta_z rotation matrices, and their first order derivation,
    // Note that it is dR/dq, but not dR/dt
    xconventionTransform(r_m[REVOLUTE_X], freedoms[REVOLUTE_X].v);
    xconventionRotation_dx(r_m_first_deriv[REVOLUTE_X], freedoms[REVOLUTE_X].v);

    yconventionTransform(r_m[REVOLUTE_Y], freedoms[REVOLUTE_Y].v);
    yconventionRotation_dy(r_m_first_deriv[REVOLUTE_Y], freedoms[REVOLUTE_Y].v);

    zconventionTransform(r_m[REVOLUTE_Z], freedoms[REVOLUTE_Z].v);
    zconventionRotation_dz(r_m_first_deriv[REVOLUTE_Z], freedoms[REVOLUTE_Z].v);

    // col major, the 12th elements is (0, 3) which stands for X translate in homogeneous coordinate
    // r_m[TRANSLATE_X] = r_m[0], it is the translate matrix for x translation freedom
    r_m[TRANSLATE_X].setIdentity();
    r_m[TRANSLATE_X].data()[12] = local_pos[0];

    // col major, the 13th elements is (1, 3) which stands for Y translate in homogeneous coordinate
    // r_m[TRANSLATE_Y] = r_m[1], the translation matrix for y freedom
    r_m[TRANSLATE_Y].setIdentity();
    r_m[TRANSLATE_Y].data()[13] = local_pos[1];

    // col major, the 14th elements is (2, 3) which stands for Z translate in homogeneous coordinate
    // r_m[TRANSLATE_Z] = r_m[1], the translation matrix for Z freedom
    r_m[TRANSLATE_Z].setIdentity();
    r_m[TRANSLATE_Z].data()[14] = local_pos[2];

    // d(translate_mat)/d[x, y, z] has only one nonzero element
    r_m_first_deriv[TRANSLATE_X].setZero();
    r_m_first_deriv[TRANSLATE_X].data()[12] = 1;

    r_m_first_deriv[TRANSLATE_Y].setZero();
    r_m_first_deriv[TRANSLATE_Y].data()[13] = 1;

    r_m_first_deriv[TRANSLATE_Z].setZero();
    r_m_first_deriv[TRANSLATE_Z].data()[14] = 1;

    // the final rotation matrix, without translation, is simple = Rz * Ry * Rx
    m = r_m[REVOLUTE_Z].topLeftCorner<3, 3>() *
        r_m[REVOLUTE_Y].topLeftCorner<3, 3>() *
        r_m[REVOLUTE_X].topLeftCorner<3, 3>();
    // std::cout << "------------------begin to output root joint rotation
    // matrix-----------------\n"; std::cout << "Rz = \n"
    // 		  << r_m[REVOLUTE_Z].topLeftCorner<3, 3>() << std::endl;
    // std::cout << "Ry = \n"
    // 		  << r_m[REVOLUTE_Y].topLeftCorner<3, 3>() << std::endl;
    // std::cout << "Rx = \n"
    // 		  << r_m[REVOLUTE_X].topLeftCorner<3, 3>() << std::endl;
    // std::cout << "final R = \n"
    // 		  << m << std::endl;
    // std::cout << "------------------end to output root joint rotation
    // matrix-----------------\n";
}

void RootJoint::Tell()
{
    std::cout << name << ":\nprev_freedom: " << total_freedoms << std::endl;
}

/**
 * \brief               Compute the d(local_transform)/dq
 * 
 *  For floating root joint, it has 6 freedoms, q = [x, y, z, theta_x, theta_y, theta_z]
 *  And the final local transform 
 *      T = Rz * Ry * Rx * Trans_z * Trans_y * Trans_x
 *      T \in R^{4 x 4}
 * 
 * 
 * dT/dn = M1 * M2 * ... * d(Mn)dn * M_{n+1} * ... * M_last
*/
void RootJoint::ComputeLocalTransformFirstDerive()
{
    /*
        dT/dx = rm[2] * rm[1] * drm/dx * rm[5] * rm[4] * rm[3]
    */
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m_first_deriv[TRANSLATE_X], r_m[5],
                         r_m[4], r_m[3], mTq[TRANSLATE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m_first_deriv[TRANSLATE_Y], r_m[0], r_m[5],
                         r_m[4], r_m[3], mTq[TRANSLATE_Y]);
    Tools::AVX4x4v1_6mat(r_m_first_deriv[TRANSLATE_Z], r_m[1], r_m[0], r_m[5],
                         r_m[4], r_m[3], mTq[TRANSLATE_Z]);

    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5], r_m[4],
                         r_m_first_deriv[REVOLUTE_X], mTq[REVOLUTE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5],
                         r_m_first_deriv[REVOLUTE_Y], r_m[3], mTq[REVOLUTE_Y]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m_first_deriv[REVOLUTE_Z],
                         r_m[4], r_m[3], mTq[REVOLUTE_Z]);
    // mTq[TRANSLATE_X] = r_m[2] * r_m[1] * r_m_first_deriv[TRANSLATE_X] *
    // r_m[5] * r_m[4] * r_m[3]; mTq[TRANSLATE_Y] = r_m[2] *
    // r_m_first_deriv[TRANSLATE_Y] * r_m[0] * r_m[5] * r_m[4] * r_m[3];
    // mTq[TRANSLATE_Z] = r_m_first_deriv[TRANSLATE_Z] * r_m[1] * r_m[0] *
    // r_m[5] * r_m[4] * r_m[3];

    // mTq[REVOLUTE_X] = r_m[2] * r_m[1] * r_m[0] * r_m[5] * r_m[4] *
    // r_m_first_deriv[REVOLUTE_X]; mTq[REVOLUTE_Y] = r_m[2] * r_m[1] * r_m[0] *
    // r_m[5] * r_m_first_deriv[REVOLUTE_Y] * r_m[3]; mTq[REVOLUTE_Z] = r_m[2] *
    // r_m[1] * r_m[0] * r_m_first_deriv[REVOLUTE_Z] * r_m[4] * r_m[3];
}

/**
 * \brief               Compute the dd(local_transform)/dqidqj
*/
void RootJoint::ComputeLocalTransformSecondDerive()
{
    xconventionRotation_dxdx(r_m_second_deriv[REVOLUTE_X],
                             freedoms[REVOLUTE_X].v);
    yconventionRotation_dydy(r_m_second_deriv[REVOLUTE_Y],
                             freedoms[REVOLUTE_Y].v);
    zconventionRotation_dzdz(r_m_second_deriv[REVOLUTE_Z],
                             freedoms[REVOLUTE_Z].v);

    r_m_second_deriv[TRANSLATE_X].setZero();
    r_m_second_deriv[TRANSLATE_Y].setZero();
    r_m_second_deriv[TRANSLATE_Z].setZero();
    mTqq[TRANSLATE_X][TRANSLATE_X].setZero();
    Tools::AVX4x4v1_6mat(r_m[2], r_m_first_deriv[1], r_m_first_deriv[0], r_m[5],
                         r_m[4], r_m[3], mTqq[TRANSLATE_Y][TRANSLATE_X]);
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m[1], r_m_first_deriv[0], r_m[5],
                         r_m[4], r_m[3], mTqq[TRANSLATE_Z][TRANSLATE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m_first_deriv[0], r_m[5], r_m[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_X][TRANSLATE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m_first_deriv[0], r_m[5],
                         r_m_first_deriv[4], r_m[3],
                         mTqq[REVOLUTE_Y][TRANSLATE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m_first_deriv[0], r_m_first_deriv[5],
                         r_m[4], r_m[3], mTqq[REVOLUTE_Z][TRANSLATE_X]);

    mTqq[TRANSLATE_Y][TRANSLATE_Y].setZero();
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m_first_deriv[1], r_m[0], r_m[5],
                         r_m[4], r_m[3], mTqq[TRANSLATE_Z][TRANSLATE_Y]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m_first_deriv[1], r_m[0], r_m[5], r_m[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_X][TRANSLATE_Y]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m_first_deriv[1], r_m[0], r_m[5],
                         r_m_first_deriv[4], r_m[3],
                         mTqq[REVOLUTE_Y][TRANSLATE_Y]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m_first_deriv[1], r_m[0], r_m_first_deriv[5],
                         r_m[4], r_m[3], mTqq[REVOLUTE_Z][TRANSLATE_Y]);

    mTqq[TRANSLATE_Z][TRANSLATE_Z].setZero();
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m[1], r_m[0], r_m[5], r_m[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_X][TRANSLATE_Z]);
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m[1], r_m[0], r_m[5],
                         r_m_first_deriv[4], r_m[3],
                         mTqq[REVOLUTE_Y][TRANSLATE_Z]);
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m[1], r_m[0], r_m_first_deriv[5],
                         r_m[4], r_m[3], mTqq[REVOLUTE_Z][TRANSLATE_Z]);

    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5], r_m[4],
                         r_m_second_deriv[3], mTqq[REVOLUTE_X][REVOLUTE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5], r_m_first_deriv[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_Y][REVOLUTE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m_first_deriv[5], r_m[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_Z][REVOLUTE_X]);

    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5], r_m_second_deriv[4],
                         r_m[3], mTqq[REVOLUTE_Y][REVOLUTE_Y]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m_first_deriv[5],
                         r_m_first_deriv[4], r_m[3],
                         mTqq[REVOLUTE_Z][REVOLUTE_Y]);

    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m_second_deriv[5], r_m[4],
                         r_m[3], mTqq[REVOLUTE_Z][REVOLUTE_Z]);
}

/**
 * \brief       compute mTqqq for root joint, compute mTqqq
 *  
 *      d^3(T)/d(qi, qj, qk), i, j, k \in {0, 1, 2, 3, 4, 5} = mTqqq[i][j][k]
 * 
 * 
*/
// extern void DerivativesAccessChained(EIGEN_VVV_tMatrixD &VVV_mat, int i, int j,
//                                      int k, const tMatrix &value);
void RootJoint::ComputeLocalTransformThirdDerive()
{
    // 1. update the third deriv for 3 rotation freedoms
    xconventionRotation_dxdxdx(r_m_third_deriv[REVOLUTE_X],
                               freedoms[REVOLUTE_X].v);
    yconventionRotation_dydydy(r_m_third_deriv[REVOLUTE_Y],
                               freedoms[REVOLUTE_Y].v);
    zconventionRotation_dzdzdz(r_m_third_deriv[REVOLUTE_Z],
                               freedoms[REVOLUTE_Z].v);
    r_m_third_deriv[TRANSLATE_X].setZero();
    r_m_third_deriv[TRANSLATE_Y].setZero();
    r_m_third_deriv[TRANSLATE_Z].setZero();

    // 2. begin to calculate mTqqq
    // T = trans_z * trans_y * trans_x * rot_z * rot_y * rot_x
    const eRootFreedomEnum
        root_freedom_order[eRootFreedomEnum::TOTAL_ROOT_FREEDOM] = {
            eRootFreedomEnum::TRANSLATE_Z, eRootFreedomEnum::TRANSLATE_Y,
            eRootFreedomEnum::TRANSLATE_X, eRootFreedomEnum::REVOLUTE_Z,
            eRootFreedomEnum::REVOLUTE_Y,  eRootFreedomEnum::REVOLUTE_X};
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
                for (int idx = 0; idx < eRootFreedomEnum::TOTAL_ROOT_FREEDOM;
                     idx++)
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
                // std::cout << "d" << (char)('x' + i) << " for d^2T/q"
                //           << (char)('x' + j) << "q" << (char)('x' + k)
                //           << " = \n"
                //           << value << std::endl;
                // if (i == 4 && j == 3 && k == 3)
                // {
                //     for (int idx = 0;
                //          idx < eRootFreedomEnum::TOTAL_ROOT_FREEDOM; idx++)
                //         std::cout << "[calc mTqqq] dof " << idx
                //                   << " derivative order "
                //                   << freedom_derivative_order[idx] << std::endl;
                //     std::cout << "dydy = " << r_m_second_deriv[1] << std::endl;
                //     exit(1);
                // }
            }
    delete[] freedom_derivative_order;
    // exit(0);
}