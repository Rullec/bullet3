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
    InitMatrix();
}

void RootJoint::GetRotations(tMatrix3d &m)
{
    for (int i = TRANSLATE_X; i < REVOLUTE_X; i++)
    {
        local_pos[i] = freedoms[i].v;
        local_pos_parent_joint[i] = freedoms[i].v;
    }
    m.setIdentity();

    xconventionTransform(r_m[REVOLUTE_X], freedoms[REVOLUTE_X].v);
    xconventionRotation_dx(r_m_first_deriv[REVOLUTE_X], freedoms[REVOLUTE_X].v);

    yconventionTransform(r_m[REVOLUTE_Y], freedoms[REVOLUTE_Y].v);
    yconventionRotation_dy(r_m_first_deriv[REVOLUTE_Y], freedoms[REVOLUTE_Y].v);

    zconventionTransform(r_m[REVOLUTE_Z], freedoms[REVOLUTE_Z].v);
    zconventionRotation_dz(r_m_first_deriv[REVOLUTE_Z], freedoms[REVOLUTE_Z].v);

    r_m[TRANSLATE_X].setIdentity();
    r_m[TRANSLATE_X].data()[12] = local_pos[0];

    r_m[TRANSLATE_Y].setIdentity();
    r_m[TRANSLATE_Y].data()[13] = local_pos[1];

    r_m[TRANSLATE_Z].setIdentity();
    r_m[TRANSLATE_Z].data()[14] = local_pos[2];

    r_m_first_deriv[TRANSLATE_X].setZero();
    r_m_first_deriv[TRANSLATE_X].data()[12] = 1;

    r_m_first_deriv[TRANSLATE_Y].setZero();
    r_m_first_deriv[TRANSLATE_Y].data()[13] = 1;

    r_m_first_deriv[TRANSLATE_Z].setZero();
    r_m_first_deriv[TRANSLATE_Z].data()[14] = 1;
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

void RootJoint::ComputeTransformFirstDerive()
{
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

void RootJoint::ComputeLocalTransformSecondDerive()
{
    xconventionRotation_dxdx(r_m_second_deriv[REVOLUTE_X],
                             freedoms[REVOLUTE_X].v);
    yconventionRotation_dydy(r_m_second_deriv[REVOLUTE_Y],
                             freedoms[REVOLUTE_Y].v);
    zconventionRotation_dzdz(r_m_second_deriv[REVOLUTE_Z],
                             freedoms[REVOLUTE_Z].v);
    // mTqq[TRANSLATE_X][TRANSLATE_X].setZero();
    // mTqq[TRANSLATE_X][TRANSLATE_Y] = r_m[2] * r_m_first_deriv[1] *
    // r_m_first_deriv[0] * r_m[5] * r_m[4] * r_m[3];
    // mTqq[TRANSLATE_X][TRANSLATE_Z] = r_m_first_deriv[2] * r_m[1] *
    // r_m_first_deriv[0] * r_m[5] * r_m[4] * r_m[3];
    // mTqq[TRANSLATE_X][REVOLUTE_X]  = r_m[2] * r_m[1] * r_m_first_deriv[0] *
    // r_m[5] * r_m[4] * r_m_first_deriv[3]; mTqq[TRANSLATE_X][REVOLUTE_Y]  =
    // r_m[2] * r_m[1] * r_m_first_deriv[0] * r_m[5] * r_m_first_deriv[4] *
    // r_m[3]; mTqq[TRANSLATE_X][REVOLUTE_Z]  = r_m[2] * r_m[1] *
    // r_m_first_deriv[0] * r_m_first_deriv[5] * r_m[4] * r_m[3];

    // mTqq[TRANSLATE_Y][TRANSLATE_Y].setZero();
    // mTqq[TRANSLATE_Y][TRANSLATE_Z] = r_m_first_deriv[2] * r_m_first_deriv[1]
    // * r_m[0] * r_m[5] * r_m[4] * r_m[3]; mTqq[TRANSLATE_Y][REVOLUTE_X]  =
    // r_m[2] * r_m_first_deriv[1] * r_m[0] * r_m[5] * r_m[4] *
    // r_m_first_deriv[3]; mTqq[TRANSLATE_Y][REVOLUTE_Y]  = r_m[2] *
    // r_m_first_deriv[1] * r_m[0] * r_m[5] * r_m_first_deriv[4] * r_m[3];
    // mTqq[TRANSLATE_Y][REVOLUTE_Z]  = r_m[2] * r_m_first_deriv[1] * r_m[0] *
    // r_m_first_deriv[5] * r_m[4] * r_m[3];

    // mTqq[TRANSLATE_Z][TRANSLATE_Z].setZero();
    // mTqq[TRANSLATE_Z][REVOLUTE_X]  = r_m_first_deriv[2] * r_m[1] * r_m[0] *
    // r_m[5] * r_m[4] * r_m_first_deriv[3]; mTqq[TRANSLATE_Z][REVOLUTE_Y]  =
    // r_m_first_deriv[2] * r_m[1] * r_m[0] * r_m[5] * r_m_first_deriv[4] *
    // r_m[3]; mTqq[TRANSLATE_Z][REVOLUTE_Z]  = r_m_first_deriv[2] * r_m[1] *
    // r_m[0] * r_m_first_deriv[5] * r_m[4] * r_m[3];

    // mTqq[REVOLUTE_X][REVOLUTE_X]  = r_m[2] * r_m[1] * r_m[0] * r_m[5] *
    // r_m[4]			   * r_m_second_deriv[3];
    // mTqq[REVOLUTE_X][REVOLUTE_Y]  = r_m[2] * r_m[1] * r_m[0] * r_m[5] *
    // r_m_first_deriv[4] * r_m_first_deriv[3]; mTqq[REVOLUTE_X][REVOLUTE_Z]  =
    // r_m[2] * r_m[1] * r_m[0] * r_m_first_deriv[5] * r_m[4] *
    // r_m_first_deriv[3];

    // mTqq[REVOLUTE_Y][REVOLUTE_Y] = r_m[2] * r_m[1] * r_m[0] * r_m[5]			 *
    // r_m_second_deriv[4] * r_m[3]; mTqq[REVOLUTE_Y][REVOLUTE_Z] = r_m[2] *
    // r_m[1] * r_m[0] * r_m_first_deriv[5] * r_m_first_deriv[4]  * r_m[3];

    // mTqq[REVOLUTE_Z][REVOLUTE_Z] = r_m[2] * r_m[1] * r_m[0] *
    // r_m_second_deriv[5] * r_m[4] * r_m[3];

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
    // mTqq[TRANSLATE_Y][TRANSLATE_X] = r_m[2] * r_m_first_deriv[1] *
    // r_m_first_deriv[0] * r_m[5] * r_m[4] * r_m[3];
    // mTqq[TRANSLATE_Z][TRANSLATE_X] = r_m_first_deriv[2] * r_m[1] *
    // r_m_first_deriv[0] * r_m[5] * r_m[4] * r_m[3];
    // mTqq[REVOLUTE_X][TRANSLATE_X]  = r_m[2] * r_m[1] * r_m_first_deriv[0] *
    // r_m[5] * r_m[4] * r_m_first_deriv[3]; mTqq[REVOLUTE_Y][TRANSLATE_X]  =
    // r_m[2] * r_m[1] * r_m_first_deriv[0] * r_m[5] * r_m_first_deriv[4] *
    // r_m[3]; mTqq[REVOLUTE_Z][TRANSLATE_X]  = r_m[2] * r_m[1] *
    // r_m_first_deriv[0] * r_m_first_deriv[5] * r_m[4] * r_m[3];

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
    // mTqq[TRANSLATE_Z][TRANSLATE_Y] = r_m_first_deriv[2] * r_m_first_deriv[1]
    // * r_m[0] * r_m[5] * r_m[4] * r_m[3]; mTqq[REVOLUTE_X][TRANSLATE_Y]  =
    // r_m[2] * r_m_first_deriv[1] * r_m[0] * r_m[5] * r_m[4] *
    // r_m_first_deriv[3]; mTqq[REVOLUTE_Y][TRANSLATE_Y]  = r_m[2] *
    // r_m_first_deriv[1] * r_m[0] * r_m[5] * r_m_first_deriv[4] * r_m[3];
    // mTqq[REVOLUTE_Z][TRANSLATE_Y]  = r_m[2] * r_m_first_deriv[1] * r_m[0] *
    // r_m_first_deriv[5] * r_m[4] * r_m[3];

    mTqq[TRANSLATE_Z][TRANSLATE_Z].setZero();
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m[1], r_m[0], r_m[5], r_m[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_X][TRANSLATE_Z]);
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m[1], r_m[0], r_m[5],
                         r_m_first_deriv[4], r_m[3],
                         mTqq[REVOLUTE_Y][TRANSLATE_Z]);
    Tools::AVX4x4v1_6mat(r_m_first_deriv[2], r_m[1], r_m[0], r_m_first_deriv[5],
                         r_m[4], r_m[3], mTqq[REVOLUTE_Z][TRANSLATE_Z]);
    // mTqq[REVOLUTE_X][TRANSLATE_Z] = r_m_first_deriv[2] * r_m[1] * r_m[0] *
    // r_m[5] * r_m[4] * r_m_first_deriv[3]; mTqq[REVOLUTE_Y][TRANSLATE_Z] =
    // r_m_first_deriv[2] * r_m[1] * r_m[0] * r_m[5] * r_m_first_deriv[4] *
    // r_m[3]; mTqq[REVOLUTE_Z][TRANSLATE_Z] = r_m_first_deriv[2] * r_m[1] *
    // r_m[0] * r_m_first_deriv[5] * r_m[4] * r_m[3];

    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5], r_m[4],
                         r_m_second_deriv[3], mTqq[REVOLUTE_X][REVOLUTE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5], r_m_first_deriv[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_Y][REVOLUTE_X]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m_first_deriv[5], r_m[4],
                         r_m_first_deriv[3], mTqq[REVOLUTE_Z][REVOLUTE_X]);
    // mTqq[REVOLUTE_X][REVOLUTE_X] = r_m[2] * r_m[1] * r_m[0] * r_m[5] * r_m[4]
    // * r_m_second_deriv[3]; mTqq[REVOLUTE_Y][REVOLUTE_X] = r_m[2] * r_m[1] *
    // r_m[0] * r_m[5] * r_m_first_deriv[4] * r_m_first_deriv[3];
    // mTqq[REVOLUTE_Z][REVOLUTE_X] = r_m[2] * r_m[1] * r_m[0] *
    // r_m_first_deriv[5] * r_m[4] * r_m_first_deriv[3];

    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m[5], r_m_second_deriv[4],
                         r_m[3], mTqq[REVOLUTE_Y][REVOLUTE_Y]);
    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m_first_deriv[5],
                         r_m_first_deriv[4], r_m[3],
                         mTqq[REVOLUTE_Z][REVOLUTE_Y]);
    // mTqq[REVOLUTE_Y][REVOLUTE_Y] = r_m[2] * r_m[1] * r_m[0] * r_m[5] *
    // r_m_second_deriv[4] * r_m[3]; mTqq[REVOLUTE_Z][REVOLUTE_Y] = r_m[2] *
    // r_m[1] * r_m[0] * r_m_first_deriv[5] * r_m_first_deriv[4]  * r_m[3];

    Tools::AVX4x4v1_6mat(r_m[2], r_m[1], r_m[0], r_m_second_deriv[5], r_m[4],
                         r_m[3], mTqq[REVOLUTE_Z][REVOLUTE_Z]);
    // mTqq[REVOLUTE_Z][REVOLUTE_Z] = r_m[2] * r_m[1] * r_m[0] *
    // r_m_second_deriv[5] * r_m[4] * r_m[3];
}
