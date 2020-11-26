#include "Joint.h"
#include "EulerAngelRotationMatrix.h"
#include "Printer.h"
// #include "BasePoint.h"
#include "tools.h"

// buffer used in jacobian calculation
static tMatrixXd local_jac_buf;

Joint::Joint(const BaseObjectParams &param)
    : BaseObject(param), mTorqueLim(0), mDiffWeight(0)
{
}

Joint::Joint(const BaseObjectJsonParam &param)
    : BaseObject(param), mTorqueLim(0), mDiffWeight(0)
{
}

void Joint::Tell()
{
    std::cout << name /*<< ":\nprev_freedom: " << total_freedoms*/ << std::endl;
    std::cout << pos << std::endl;
    // std::cout << "parent_joint: " << parent_joint->GetName() << std::endl;
}

Freedom *Joint::AddFreedom(Freedom &f)
{
    Freedom fs(f);
    freedoms.push_back(fs);
    local_freedom = static_cast<int>(freedoms.size());
    return &freedoms[freedoms.size() - 1];
}

Freedom *Joint::GetFreedoms(int order)
{
    assert(order < freedoms.size() && order >= 0);
    return &freedoms[order];
}

Freedom *Joint::GetFreedomByAxis(tVector3d axis, int type)
{
    if (freedoms.empty())
        return nullptr;
    for (auto &f : freedoms)
    {
        if (f.axis == axis && f.type == type)
        {
            return &f;
        }
    }
    return nullptr;
}

/**
 * \brief					Init some variables and the relationship between parent and child
 */
void Joint::InitTerms()
{
    if (parent)
    {
        InitPrevFreedomIds();
        prev_freedoms = parent->GetNumTotalFreedoms();
        if (!parent_joint)
            parent_joint = parent->GetParent();
        const tMatrix &m_neg_ini = parent_joint->GetNegInitRotationtMatrix();
        // rot_parent_child = m_neg_ini * init_rotation_matrix_4x4;
        Tools::AVX4x4v1(m_neg_ini, init_rotation_matrix_4x4, rot_parent_child);
        local_pos_parent_joint = Tools::GettVector3d(
            parent->GetLocalTransform() * Tools::GettVector(local_pos));
        // local_pos_parent_joint = local_pos *2;
        // before 2020/04/08 local pos is from this joint to parent joint
        // but now we modify the kinematics chain, local pos will be set to the
        // parent joint
    }
    else
    {
        rot_parent_child.setIdentity();
        local_pos_parent_joint = local_pos;
    }
    for (auto &f : freedoms)
    {
        dependent_dof_id.push_back(f.id);
    }
    InitGlobalToTotalFreedomMap();
    total_freedoms = prev_freedoms + GetNumOfFreedom();
    local_jac_buf.resize(3, total_freedoms);
    InitMatrix();
}

/**
 * \brief					��ʼ����ǰ�ؽڵĸ�����������
 */
void Joint::InitMatrix()
{
    r_m.resize(local_freedom);
    r_m_first_deriv.resize(local_freedom);
    r_m_second_deriv.resize(local_freedom);
    r_m_third_deriv.resize(local_freedom);
    mTq.resize(local_freedom, tMatrix::Zero());
    mWq.resize(total_freedoms, tMatrix::Zero());

    JK_w_local.resize(3, local_freedom);
    JK_w.resize(3, global_freedom);
    JK_v.resize(3, global_freedom);
    JK.resize(6, global_freedom);
    JK_w.setZero();
    JK_v.setZero();
    JK.setZero();

    JK.block(3, 0, 3, global_freedom);

    // JK_v_dot.resize(3, global_freedom);
    // JK_w_dot.resize(3, global_freedom);
    // JK_dot.resize(6 , global_freedom);
    // JK_dot.setZero();

    // jkv_dq.resize(global_freedom, tMatrixXd::Zero(3, global_freedom));
    // jkw_dq.resize(global_freedom, tMatrixXd::Zero(3, global_freedom));

    // =================second order===============
    mTqq.resize(local_freedom);

    for (int i = 0; i < local_freedom; ++i)
    {
        mTqq[i].resize(i + 1);
        for (auto &x : mTqq[i])
            x.setZero();
    }

    mWqq.resize(total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        mWqq[i].resize(i + 1);
        for (auto &x : mWqq[i])
            x.setZero();
    }

    // ==================third order================
    mTqqq.resize(local_freedom);
    for (int i = 0; i < local_freedom; i++)
    {

        mTqqq[i].resize(i + 1);
        for (int j = 0; j <= i; j++)
        {
            // std::cout << "mTqqq " << i << " " << j << " resize\n";
            mTqqq[i][j].resize(j + 1, tMatrix::Zero());
        }
    }
    mWqqq.resize(total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        mWqqq[i].resize(i + 1);
        for (int j = 0; j <= i; j++)
        {
            // std::cout << "mWqqq " << i << " " << j << " resize\n";
            mWqqq[i][j].resize(j + 1, tMatrix::Zero());
        }
    }
}

/**
 * \brief					update the status of joint
 * \param compute_gradient	update the gradient? no matter 1 or 2 or 3 order
 */
void Joint::UpdateState(bool compute_gradient)
{
    // 1. compute orientation
    GetRotations(orientation);
    // 2. compute local transform
    ComputeLocalTransform();
    // 3. compute global transform
    ComputeGlobalTransform();
    // 4. compute derivatives
    if (compute_gradient)
    {
        // first order values are computed default
        ComputeLocalTransformFirstDerive();
        ComputeGlobalTransformFirstDerive();

        ComputeLocalJkw();
        ComputeJKv();
        ComputeJKw();
        ComputeJK();

        // second order derivatives need another switch
        if (compute_second_derive)
        {
            ComputeLocalSecondDeriveMatrix();
            ComputeLocalTransformSecondDerive();
            ComputeGlobalTransformSecondDerive();
        }

        // third order derivatives need another switch
        if (compute_third_derive)
        {
            if (compute_second_derive == false)
            {
                std::cout << "[error] Joint::UpdateState: 3rd derivatives can "
                             "only be "
                             "calculated when 2ed derivatives are availiable\n";
                exit(1);
            }
            ComputeLocalThirdDeriveMatrix();
            ComputeLocalTransformThirdDerive();
            ComputeGlobalTransformThirdDerive();
        }
    }
}

/**
 * \brief
 * ���µ�ǰ���󣬼�����ת����
 */
void Joint::UpdateMatrix()
{
    int count = 0;
    for (auto &f : freedoms)
    {
        if (f.axis[0] == 1)
        {
            xconventionTransform(r_m[count], f.v);
            // xconventionRotation_dx(r_m_first_deriv[count], f.v);
        }
        else if (f.axis[1] == 1)
        {
            yconventionTransform(r_m[count], f.v);
            // yconventionRotation_dy(r_m_first_deriv[count], f.v);
        }
        else if (f.axis[2] == 1)
        {
            zconventionTransform(r_m[count], f.v);
            // zconventionRotation_dz(r_m_first_deriv[count], f.v);
        }
        count++;
    }
}

/**
 * \brief       Compute the first derivatives of rot mat \frac{\partial R_L}{\partial q}
 */
void Joint::ComputeLocalFirstDeriveMatrix()
{
    int count = 0;
    for (auto &f : freedoms)
    {
        if (f.axis[0] == 1)
        {
            xconventionRotation_dx(r_m_first_deriv[count], f.v);
        }
        else if (f.axis[1] == 1)
        {
            yconventionRotation_dy(r_m_first_deriv[count], f.v);
        }
        else if (f.axis[2] == 1)
        {
            zconventionRotation_dz(r_m_first_deriv[count], f.v);
        }
        count++;
    }
}

/**
 * \brief       compute the second order derivatives of rotation matrix \frac{\partial^2 R_L}{\partial q_i \partial q_j}
 */
void Joint::ComputeLocalSecondDeriveMatrix()
{
    int count = 0;
    for (auto &f : freedoms)
    {
        if (f.axis[0] == 1)
        {
            xconventionRotation_dxdx(r_m_second_deriv[count], f.v);
        }
        else if (f.axis[1] == 1)
        {
            yconventionRotation_dydy(r_m_second_deriv[count], f.v);
        }
        else if (f.axis[2] == 1)
        {
            zconventionRotation_dzdz(r_m_second_deriv[count], f.v);
        }
        count++;
    }
}

/**
 * \brief       compute the third order derivatives of the rot mat
 * 
*/
void Joint::ComputeLocalThirdDeriveMatrix()
{
    int count = 0;
    for (auto &f : freedoms)
    {
        if (f.axis[0] == 1)
        {
            xconventionRotation_dxdxdx(r_m_third_deriv[count], f.v);
        }
        else if (f.axis[1] == 1)
        {
            yconventionRotation_dydydy(r_m_third_deriv[count], f.v);
        }
        else if (f.axis[2] == 1)
        {
            zconventionRotation_dzdzdz(r_m_third_deriv[count], f.v);
        }
        count++;
    }
}

/**
 * \brief					Get the local orientation of this joint
 * \param m					local orientation of this joint
 */
void Joint::GetRotations(tMatrix3d &m)
{
    m.setIdentity();
    UpdateMatrix();
    for (auto &rotation_matrix : r_m)
    {
        m = rotation_matrix.topLeftCorner<3, 3>() * m;
    }
}

tMatrix3d Joint::GetRotations()
{
    tMatrix3d m;
    GetRotations(m);
    return m;
}
/**
 * \brief					compute mTq, d(local_rot)/dq, 
 *                          local_rot = Rz * Ry * Rx
 */
void Joint::ComputeLocalTransformFirstDerive()
{
    // 1. first calculate the dRx/dqx, dRy/dqy, dRz/dqz
    ComputeLocalFirstDeriveMatrix();

    /*
        2. T is the local transform matrix
        T = Rz * Ry * Rx
        mTq = [dT/dqx, dT/dqy, dT/dqz]
    */
    if (mTq.size() == 1)
    {
        mTq[0] = r_m_first_deriv[0];
    }
    else if (mTq.size() == 2)
    {
        // mTq[0] = r_m[1] * r_m_first_deriv[0];
        // mTq[1] = r_m_first_deriv[1] * r_m[0];
        Tools::AVX4x4v1(r_m[1], r_m_first_deriv[0], mTq[0]);
        Tools::AVX4x4v1(r_m_first_deriv[1], r_m[0], mTq[1]);
    }
    else if (mTq.size() == 3)
    {
        // mTq[0] = r_m[2] * r_m[1] * r_m_first_deriv[0];
        // mTq[1] = r_m[2] * r_m_first_deriv[1] * r_m[0];
        // mTq[2] = r_m_first_deriv[2] * r_m[1] * r_m[0];

        Tools::AVX4x4v1_3mat(r_m[2], r_m[1], r_m_first_deriv[0], mTq[0]);
        Tools::AVX4x4v1_3mat(r_m[2], r_m_first_deriv[1], r_m[0], mTq[1]);
        Tools::AVX4x4v1_3mat(r_m_first_deriv[2], r_m[1], r_m[0], mTq[2]);
    }
}

/**
 * \brief					ͨCalculate mWq from mTq.
 * 
 * T is the local rotation matrix, T_local = Rz * Ry * Rx
 * W is the world rotation matrix, W = W_parent * T_child_to_parent * T_local
 * 
 * this function tries to calculate dW/dq, q is all dependent freedom of this joint
 * For prev freedoms: dW/dq = d(W_parent)/dq * T_child_to_parnet * T_local
 * For joint's local freedoms: dW/dq = W_parent * T_child_to_parent * dT_local/dq
 */
void Joint::ComputeGlobalTransformFirstDerive()
{
    for (int i = 0; i < prev_freedoms; i++)
    { // for root joint prev_freedom is 0, so this part will not be executed
        if (parent_joint)
        {
            const tMatrix &parent_mwqi = parent_joint->GetMWQ(i);
            // mWq[i] = parent_joint->GetMWQ(i) * rot_parent_child *
            // local_transform;
            Tools::AVX4x4v1_3mat(parent_mwqi, rot_parent_child, local_transform,
                                 mWq[i]);
        }
    }
    for (int i = 0; i < local_freedom; i++)
    {
        if (parent_joint)
        {
            const tMatrix &m = parent_joint->GetGlobalTransform();
            // mWq[i + prev_freedoms] = m * rot_parent_child * mTq[i];
            Tools::AVX4x4v1_3mat(m, rot_parent_child, mTq[i],
                                 mWq[i + prev_freedoms]);
        }
        else
        {
            // mWq[i + prev_freedoms] = init_rotation_matrix_4x4 * mTq[i];
            Tools::AVX4x4v1(init_rotation_matrix_4x4, mTq[i],
                            mWq[i + prev_freedoms]);
        }
    }
}

/**
 * \brief       compute the second order derivation of
 *              local transform matrix T w.r.t q
 *	    mTqq = \frac{\partial^2 R_L}{\partial q_i \partial q_j}
 */
void Joint::ComputeLocalTransformSecondDerive()
{
    tMatrix t = tMatrix::Zero();
    for (int i = 0; i < local_freedom; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            mTqq[i][j].setIdentity();

            for (int k = 0; k < local_freedom; k++)
            {
                if (k == i && i != j)
                {
                    // mTqq[i][j] = r_m_first_deriv[k] * mTqq[i][j];
                    Tools::AVX4x4v1(r_m_first_deriv[k], mTqq[i][j], t);
                    mTqq[i][j] = t;
                }
                else if (k == i && i == j)
                {
                    // mTqq[i][j] = r_m_second_deriv[k] * mTqq[i][j];
                    Tools::AVX4x4v1(r_m_second_deriv[k], mTqq[i][j], t);
                    mTqq[i][j] = t;
                }
                else if (k != i && k == j)
                {
                    // mTqq[i][j] = r_m_first_deriv[k] * mTqq[i][j];
                    Tools::AVX4x4v1(r_m_first_deriv[k], mTqq[i][j], t);
                    mTqq[i][j] = t;
                }
                else
                {
                    // mTqq[i][j] = r_m[k] * mTqq[i][j];
                    Tools::AVX4x4v1(r_m[k], mTqq[i][j], t);
                    mTqq[i][j] = t;
                }
            }
        }
    }
}

/**
 * \brief           compute mTqqq, d^3(T)/d(qi, qj, qk), i, j, k \in {x, y, x} = mTqqq[i][j][k]
 *  This derivatives can be grouped into 3 types:
 *  1. i!=j, i!=k, j!=k: 9 cases, compute 1 time, write 6 times
 * 
 *  2. (i==j or i==k or j==k) but the left two are different: 18 cases
 *      equal index can be x or y or z. and the diff can be (yz) or (xy) or (xy)
 *      computation 6 times, write 18 times
 *  3. (i==j==k) 3 cases
 *      all is equal to x, y, or z. computation 3 times, write 3 times
 *  The above statement is about spherical joint. For revolute joint there is only one entry
 *
*/
void Joint::ComputeLocalTransformThirdDerive()
{
    switch (GetJointType())
    {
    case JointType::FIXED_JOINT:
    {
        return;
    }
    break;
    case JointType::SPHERICAL_JOINT:
    {
        ComputeLocalTransformThirdDeriveSpherical();
    }
    break;
    case JointType::REVOLUTE_JOINT:
    {
        mTqqq[0][0][0] = r_m_third_deriv[0];
    }
    break;
    default:
        std::cout
            << "[error] Joint compute 3rd derivatives unsupport joint type "
            << GetJointType() << std::endl;
        exit(1);
        break;
    }
}

/**
 * \brief					Compute the 2ed order derivation of global trans mat w.r.t freedoms
 *							mWqq = \frac{\partial^2R_W}{\partial q_i \partial q_j}
 */
void Joint::ComputeGlobalTransformSecondDerive()
{
    const int num_parent_dofs = prev_freedoms;

    tMatrix parent_t_child;
    if (parent)
        parent_t_child = rot_parent_child;

    for (int i = 0; i < num_parent_dofs; i++)
    {
        for (int j = 0; j <= i; j++)
        {
            // mWqq[i][j].noalias() = parent_joint->GetMWQQ(i, j) *
            // parent_t_child * local_transform;
            Tools::AVX4x4v1_3mat(parent_joint->GetMWQQ(i, j), parent_t_child,
                                 local_transform, mWqq[i][j]);
        }
        // for (int j = 0; j < local_freedom; j++)	{
        //	if (parent_joint)
        //		mWqq[i][j + num_parent_dofs] = parent_joint->GetMWQ(i) *
        // parent_t_child * mTq[j]; 	else 		mWqq[i][j +
        // num_parent_dofs] = init_rotation_matrix_4x4 * mTq[j];
        //}
    }

    for (int i = 0; i < local_freedom; i++)
    {
        for (int j = 0; j < num_parent_dofs; j++)
        {
            if (parent_joint)
            {
                // mWqq[num_parent_dofs + i][j].noalias() =
                // parent_joint->GetMWQ(j) * parent_t_child * mTq[i];
                const tMatrix &m = parent_joint->GetMWQ(j);
                Tools::AVX4x4v1_3mat(m, parent_t_child, mTq[i],
                                     mWqq[num_parent_dofs + i][j]);
            }
            else
            {
                // mWqq[num_parent_dofs + i][j].noalias() =
                // init_rotation_matrix_4x4 * mTq[i];
                Tools::AVX4x4v1(init_rotation_matrix_4x4, mTq[i],
                                mWqq[num_parent_dofs + i][j]);
            }
        }

        for (int j = 0; j <= i; j++)
        {
            if (parent_joint)
            {
                // mWqq[num_parent_dofs + i][num_parent_dofs + j].noalias() =
                // parent_joint->GetGlobalTransform() * rot_parent_child *
                // mTqq[i][j];
                const tMatrix &m = parent_joint->GetGlobalTransform();
                Tools::AVX4x4v1_3mat(
                    m, rot_parent_child, mTqq[i][j],
                    mWqq[num_parent_dofs + i][num_parent_dofs + j]);
            }
            else
            {
                // mWqq[num_parent_dofs + i][num_parent_dofs + j].noalias() =
                // init_rotation_matrix_4x4 * mTqq[i][j];
                Tools::AVX4x4v1(init_rotation_matrix_4x4, mTqq[i][j],
                                mWqq[num_parent_dofs + i][num_parent_dofs + j]);
            }
        }
    }
}

/**
 * \brief					calculate the jacobian in given position 
 * \param j					jacobian buffer output
 * \param p					local offset of targeted point
 */
void Joint::ComputeJacobiByGivenPoint(tMatrixXd &j, const tVector &p) const
{
    if (j.rows() != 3 || j.cols() != total_freedoms)
        j.resize(3, total_freedoms);

    tVector column;
    for (size_t i = 0; i < mWq.size(); i++)
    {
        Tools::MatMul4x1(mWq[i], p, column);
        // j.col(i) = Tools::GettVector3d(column);
        j.col(i).noalias() = column.segment(0, 3);
    }
}

void Joint::ComputeHessianByGivenPoint(EIGEN_V_MATXD &ms,
                                       const tVector &p) const
{
    ms.resize(3, tMatrixXd::Zero(total_freedoms, total_freedoms));
    assert(total_freedoms == static_cast<int>(mWqq.size()));

    for (size_t i = 0; i < mWqq.size(); i++)
    {
        for (int j = 0; j < total_freedoms; ++j)
        {
            tVector column = mWqq[i][j] * p;
            for (size_t k = 0; k < ms.size(); ++k)
            {
                ms[k].data()[i * total_freedoms + j] = column[k];
            }
        }
    }
}

/**
 * \brief			Compute Jacobian (Jv) for this point p with respect to the whole DOF of this skeleton 
 * \param j			the reference jacobian mat, it will be revised in the function
 * \param p			the target point where we want to get the jacobian(Jv), expressed in joint local frame
 */
void Joint::ComputeJacobiByGivenPointTotalDOF(tMatrixXd &j,
                                              const tVector &p) const
{
    // 1. get the dependent Jacobian
    ComputeJacobiByGivenPoint(local_jac_buf, p);
    // std::cout <<"Joint::Compute short = \n" << j_dependent_dof << std::endl;
    assert(local_jac_buf.cols() == this->total_freedoms);

    // 2. map this jacobian to global jacobian
    if (j.cols() != 3 || j.rows() != global_freedom)
        j.resize(3, global_freedom);
    j.setZero();
    // std::cout <<"prev size = " << prev_freedoms <<", freedoms size = " <<
    // freedoms.size() << std::endl;

    /* 	Attention: the size of prev_freedoms_id should be equal to prev_freedoms
            But for root joint, this law is falsely broken. It does not effect
       other behavior but just a bug.
    */
    assert((prev_freedoms + freedoms.size()) == total_freedoms);

    // 2.1 map other dependent freedom
    for (int i = 0; i < prev_freedoms; i++)
    {
        j.col(dependent_dof_id[i]) = local_jac_buf.col(i);
        // j.col(i) = j_dependent_dof.col(prev_freedom_id[i]);
    }

    // 2.2 map myself dependent freedom
    for (int i = 0; i < freedoms.size(); i++)
    {
        j.col(freedoms[i].id) = local_jac_buf.col(prev_freedoms + i);
    }
}

// void Joint::ComputeJK_dot() {
//	JK_dot.block(0, 0, 3, global_freedom) = JK_v_dot;
//	JK_dot.block(3, 0, 3, global_freedom) = JK_w_dot;
//}

void Joint::ComputeJKv_dot(const tVectorXd &q_dot, const tVector3d &p)
{
    Printer::Error("Calling ComputeJKv_dot in Joint");
    exit(-1);
    JK_v_dot.setZero();
    for (size_t i = 0; i < mWqq.size(); i++)
    {
        jkv_dq[i].setZero();
        for (int j = 0; j < total_freedoms; ++j)
        {
            const tVector column = mWqq[i][j] * Tools::GettVector(p);
            jkv_dq[i].col(dependent_dof_id[j]) = Tools::GettVector3d(column);
        }
        JK_v_dot += jkv_dq[i] * q_dot[i];
    }
}

void Joint::ComputeJKw_dot(const tVectorXd &q_dot)
{
    Printer::Error("Calling ComputeJKw_dot in Joint");
    exit(-1);
    JK_w_dot.setZero();
    for (size_t i = 0; i < mWqq.size(); ++i)
    {
        jkw_dq[i].setZero();
        for (int j = 0; j < total_freedoms; ++j)
        {
            const tMatrix3d m =
                mWqq[i][j].topLeftCorner<3, 3>() *
                    global_transform.topLeftCorner<3, 3>().transpose() +
                mWq[j].topLeftCorner<3, 3>() *
                    mWq[i].topLeftCorner<3, 3>().transpose();
            const tVector3d column = Tools::FromSkewSymmetric(m);
            jkw_dq[i].col(dependent_dof_id[j]) = column;
        }
        JK_w_dot += jkw_dq[i] * q_dot[dependent_dof_id[i]];
    }
}

void Joint::ComputeLocalTransform()
{
    local_transform.topLeftCorner<3, 3>() = orientation;
    local_transform.data()[12] = local_pos_parent_joint.data()[0]; // *2;
    local_transform.data()[13] = local_pos_parent_joint.data()[1]; // *2;
    local_transform.data()[14] = local_pos_parent_joint.data()[2]; // *2;
    local_transform.data()[15] = 1.;
}

/**
 * \brief					���� global_transform ����
 */
void Joint::ComputeGlobalTransform()
{
    if (parent_joint)
    {
        const tMatrix &m = parent_joint->GetGlobalTransform();
        // global_transform = m * rot_parent_child * local_transform;
        Tools::AVX4x4v1_3mat(m, rot_parent_child, local_transform,
                             global_transform);
    }
    else
    {
        // global_transform = init_rotation_matrix_4x4 * local_transform;
        Tools::AVX4x4v1(init_rotation_matrix_4x4, local_transform,
                        global_transform);
    }
    tVector p(0, 0, 0, 1);
    tVector p_;
    Tools::MatMul4x1(global_transform, p, p_);
    pos[0] = p_[0];
    pos[1] = p_[1];
    pos[2] = p_[2];
}

const tMatrix &Joint::GetMWQQ(int i, int j) const
{
    int r = std::max(i, j);
    int c = std::min(i, j);
    return mWqq[r][c];
}

const tMatrixXd &Joint::GetJKDot() const
{
    Printer::Error("Calling GetJKDot in Joint");
    exit(-1);
    return mWqq[0][0];
}

int Joint::GetNumOfFreedom() const { return static_cast<int>(freedoms.size()); }

/**
 * \brief           Set a single freedom value of this joint
 * \param id        the LOCAL id of target dof we want to set, beginning from 0 -> DOF of this joint
 *                  NOT GLOBAL!
*/
void Joint::SetFreedomValue(int id, double v)
{
    if (id >= freedoms.size())
    {
        printf("[error] Joint %d SetFreedomValue id %d exceed the dof num %d\n",
               this->mId, id, GetNumOfFreedom());
        exit(1);
    }
    freedoms[id].v = v;
}

/**
 * \brief           Get a single freedom value of this joint
 * \param id        the LOCAL id of target dof we want to get, beginning from 0 -> DOF of this joint
 *                  NOT GLOBAL!
*/
void Joint::GetFreedomValue(int id, double &v) const
{
    if (id >= freedoms.size())
    {
        printf("[error] Joint %d GetFreedomValue id %d exceed the dof num %d\n",
               this->mId, id, GetNumOfFreedom());
        exit(1);
    }
    v = freedoms[id].v;
}

/**
 * \brief           Set all of the freedom values of this joint
 * \param v         a vector whose length is the same as the DOF number of this joint
*/
void Joint::SetFreedomValue(std::vector<double> &v)
{
    if (v.size() != freedoms.size())
    {
        printf("[error] Joint %d SetFreedomValue size %d != the dof num %d\n",
               this->mId, v.size(), GetNumOfFreedom());
        exit(1);
    }
    for (int i = 0; i < GetNumOfFreedom(); i++)
    {
        this->freedoms[i].v = v[i];
    }
}

/**
 * \brief           Get all of the freedom values of this joint
 * \param v         a ref vector, it will be changed in this function
*/
void Joint::GetFreedomValue(std::vector<double> &v) const
{
    v.resize(freedoms.size());
    for (int i = 0; i < GetNumOfFreedom(); i++)
    {
        v[i] = freedoms[i].v;
    }
}

/**
 * \brief           check if this joint is a root joint
 * \return true if it's
*/
bool Joint::GetIsRootJoint() const
{
    bool should_be_root = -1 == GetParentId();

    // double check the joint type is root
    if (should_be_root == true)
    {
        BTGEN_ASSERT(joint_type == JointType::NONE_JOINT ||
                     joint_type == JointType::BIPEDAL_NONE_JOINT ||
                     joint_type == JointType::LIMIT_NONE_JOINT ||
                     joint_type == JointType::FIXED_NONE_JOINT);
    }
    return should_be_root;
}

void Joint::CleanGradient()
{
    for (auto &f : freedoms)
    {
        f.clear_grad();
    }
}

// void Joint::SetJointVel(const tVector3d &vel_) { mJointVel = vel_; }
// void Joint::SetJointOmega(const tVector3d &omega_) { mJointOmega = omega_; }
// tVector3d Joint::GetJointVel() const { return mJointVel; }

// tVector3d Joint::GetJointOmega() const { return mJointOmega; }
void Joint::SetTorqueLim(double lim) { mTorqueLim = lim; }

double Joint::GetTorqueLim() const { return mTorqueLim; }

void Joint::SetDiffWeight(double weight) { mDiffWeight = weight; }
double Joint::GetDiffWeight() const { return mDiffWeight; }

/** \brief           compute spherical joint mTqqq, d^3(T)/d(qi, qj, qk), i, j, k \in {x, y, x} = mTqqq[i][j][k]
 *  This derivatives can be grouped into 3 types:
 *  group 1. i!=j, i!=k, j!=k: 9 cases, compute 1 time, write 6 times
 * 
 *  group 2. (i==j or i==k or j==k) but the left two are different: 18 times
 *      equal index can be x or y or z. and the diff can be (yz) or (xy) or (xy)
 *      computation 6 times, write 18 times
 *  group 3. (i==j==k) 3 cases
 *      all is equal to x, y, or z. computation 3 times, write 3 times
 */
void Joint::ComputeLocalTransformThirdDeriveSpherical()
{
    // 1. check joint type
    if (joint_type != JointType::SPHERICAL_JOINT)
    {
        std::cout
            << "[error] Joint::compute local transform third derive spherical "
               "should not be called when joint type is "
            << joint_type << std::endl;
        exit(1);
    }

    // 2. handle the group 1
    tMatrix buffer;
    {
        // computation
        Tools::AVX4x4v1_3mat(r_m_first_deriv[2], r_m_first_deriv[1],
                             r_m_first_deriv[0], buffer);

        // write for 6 times
        // memcpy(mTqqq[0][1][2].data(), buffer.data(), sizeof(double) * 16);
        // memcpy(mTqqq[0][2][1].data(), buffer.data(), sizeof(double) * 16);
        // memcpy(mTqqq[1][0][2].data(), buffer.data(), sizeof(double) * 16);
        // memcpy(mTqqq[1][2][0].data(), buffer.data(), sizeof(double) * 16);
        memcpy(mTqqq[2][1][0].data(), buffer.data(), sizeof(double) * 16);
        // memcpy(mTqqq[2][0][1].data(), buffer.data(), sizeof(double) * 16);
    }

    // 3. handle the group 2
    // T = Rz * Ry * Rx
    { // 3.1 dT/(dqz dqz dqx), Ry = Ry

        {
            buffer.noalias() =
                r_m_second_deriv[2] * r_m[1] * r_m_first_deriv[0];
            memcpy(mTqqq[2][2][0].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[2][0][2].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[0][2][2].data(), buffer.data(), sizeof(double) * 16);
        }
        // 3.2 dT/(dqz dqz dqy), Rx = Rx
        {
            buffer.noalias() =
                r_m_second_deriv[2] * r_m_first_deriv[1] * r_m[0];
            memcpy(mTqqq[2][2][1].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[2][1][2].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[1][2][2].data(), buffer.data(), sizeof(double) * 16);
        }
        // 3.1 dT/(dqy dqy dqx), Ry = Ry
        {
            buffer.noalias() =
                r_m[2] * r_m_second_deriv[1] * r_m_first_deriv[0];
            memcpy(mTqqq[1][1][0].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[1][0][1].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[0][1].data(), buffer.data(), sizeof(double) * 16);
        }
        // 3.2 dT/(dqy dqy dqz), Rx = Rx
        {
            buffer.noalias() =
                r_m_first_deriv[2] * r_m_second_deriv[1] * r_m[0];
            // memcpy(mTqqq[1][1][2].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[1][2][1].data(), buffer.data(), sizeof(double) * 16);
            memcpy(mTqqq[2][1][1].data(), buffer.data(), sizeof(double) * 16);
        }
        // 3.1 dT/(dqx dqx dqy)
        {
            buffer.noalias() =
                r_m[2] * r_m_first_deriv[1] * r_m_second_deriv[0];
            // memcpy(mTqqq[0][0][1].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[0][1][0].data(), buffer.data(), sizeof(double) * 16);
            memcpy(mTqqq[1][0][0].data(), buffer.data(), sizeof(double) * 16);
        }

        // 3.2 dT/(dqx dqx dqz)
        {
            buffer.noalias() =
                r_m_first_deriv[2] * r_m[1] * r_m_second_deriv[0];
            // memcpy(mTqqq[0][0][2].data(), buffer.data(), sizeof(double) * 16);
            // memcpy(mTqqq[0][2][0].data(), buffer.data(), sizeof(double) * 16);
            memcpy(mTqqq[2][0][0].data(), buffer.data(), sizeof(double) * 16);
        }
    }

    // 4. handle the group 3
    // dT^3/d^3qz
    buffer.noalias() = r_m_third_deriv[2] * r_m[1] * r_m[0];
    memcpy(mTqqq[2][2][2].data(), buffer.data(), sizeof(double) * 16);

    // dT^3/d^3qy
    buffer.noalias() = r_m[2] * r_m_third_deriv[1] * r_m[0];
    memcpy(mTqqq[1][1][1].data(), buffer.data(), sizeof(double) * 16);

    // dT^3/^3qz
    buffer.noalias() = r_m[2] * r_m[1] * r_m_third_deriv[0];
    memcpy(mTqqq[0][0][0].data(), buffer.data(), sizeof(double) * 16);
}

/**
 * \brief               Compute the third derivatives of world transform matrix W w.r.t q, aka mWqqq
 *          
 *  W is the local to world transform matrix
 *      W = W_parent * T_child_to_parent * T_local     
 * 
 * so that, mWqqq[i][j][k] = d(W)/d(qi, qj, qk), i,j,k \in [0, 1, 2, ..., total_freedoms]
 *  
 *  total_freedoms = prev_freedoms(ancestors' freedom) + local_freedoms(myself's freedom)
 * if i, j, k    
*/
void Joint::ComputeGlobalTransformThirdDerive()
{
    tMatrix child_to_parent;
    if (parent == nullptr)
        child_to_parent = init_rotation_matrix_4x4;
    else
        child_to_parent = rot_parent_child;
    // visit the upper tensor
    tMatrix buffer;
    // the dof index "i" is in the range of my own dependent freedoms, [0, total_freedoms - 1]
    for (int i = total_freedoms - 1; i >= 0; i--)
        for (int j = i; j >= 0; j--)
            for (int k = j; k >= 0; k--)
            {
                // k <= j <= i

                if (i < prev_freedoms)
                {
                    // all derivatives are about previous freedoms
                    buffer.noalias() = parent_joint->GetMWQQQ(i, j, k) *
                                       child_to_parent * local_transform;
                }
                else if (j < prev_freedoms)
                {
                    // k, j are about previous freedoms
                    // i is self freedoms
                    buffer.noalias() = parent_joint->GetMWQQ(k, j) *
                                       child_to_parent *
                                       GetMTQ(i - prev_freedoms);
                }
                else if (k < prev_freedoms)
                {
                    // k is about previous freedoms
                    // i, j is self freedoms
                    buffer.noalias() =
                        parent_joint->GetMWQ(k) * child_to_parent *
                        GetMTQQ(i - prev_freedoms, j - prev_freedoms);
                }
                else
                {
                    // i, j, k is all about self freedoms
                    if (parent) // has parent
                    {
                        buffer.noalias() =
                            parent_joint->GetGlobalTransform() *
                            child_to_parent *
                            GetMTQQQ(i - prev_freedoms, j - prev_freedoms,
                                     k - prev_freedoms);
                    }
                    else // no parent
                    {
                        buffer.noalias() =
                            init_rotation_matrix_4x4 * child_to_parent *
                            GetMTQQQ(i - prev_freedoms, j - prev_freedoms,
                                     k - prev_freedoms);
                    }
                }

                mWqqq[i][j][k].noalias() = buffer;
            }
}

/**
 * \brief           Please check the definition of mWqqq 
*/
const tMatrix &Joint::GetMWQQQ(int i, int j, int k) const
{
    if (i < j)
        btMathUtil::Swap(i, j);
    if (i < k)
        btMathUtil::Swap(i, k);
    if (j < k)
        btMathUtil::Swap(j, k);
    // sort: i >= j >= k
    return mWqqq[i][j][k];
}
const tMatrix &Joint::GetMTQ(int i) const
{
    if (i >= local_freedom)
    {
        std::cout << "[error] Joint mTQ idx " << i << " >= local freedom "
                  << local_freedom << std::endl;
        exit(1);
    }
    return mTq[i];
}

const tMatrix &Joint::GetMTQQ(int i, int j) const
{
    if (i < j)
        btMathUtil::Swap(i, j);
    if (i >= local_freedom)
    {
        std::cout << "[error] Joint mTQQ idx " << i << " >= local freedom "
                  << local_freedom << std::endl;
        exit(1);
    }

    return mTqq[i][j];
}
const tMatrix &Joint::GetMTQQQ(int i, int j, int k) const
{
    if (i < j)
        btMathUtil::Swap(i, j);
    if (i < k)
        btMathUtil::Swap(i, k);
    if (j < k)
        btMathUtil::Swap(j, k);
    if (i >= local_freedom)
    {
        std::cout << "[error] Joint mTQQQ idx " << i << " >= local freedom "
                  << local_freedom << std::endl;
        exit(1);
    }
    return mTqqq[i][j][k];
}

/**
 * \brief           Set the freedom's velocity value "vdot". 
 * they are elements in generalize velocity 
 * \param dof_id    the same as in SetFreedomValue
*/
void Joint::SetFreedomValueDot(int dof_id, double vdot)
{
    if (dof_id >= freedoms.size())
    {
        printf(
            "[error] Joint %d SetFreedomValueDot id %d exceed the dof num %d\n",
            mId, dof_id, GetNumOfFreedom());
        exit(1);
    }
    freedoms[dof_id].vdot = vdot;
}

/**
 * \brief           Get the freedom's velocity value "vdot"
 * \param dof_id    the same as in SetFreedomValue
*/
void Joint::GetFreedomValueDot(int dof_id, double &vdot) const
{
    if (dof_id >= freedoms.size())
    {
        printf(
            "[error] Joint %d GetFreedomValueDot id %d exceed the dof num %d\n",
            mId, dof_id, GetNumOfFreedom());
        exit(1);
    }
    vdot = freedoms[dof_id].vdot;
}

/**
 * \brief           Set the freedom's velocity value vector "vdot"
 * \param vdot      the same as in SetFreedomValue
*/
void Joint::SetFreedomValueDot(std::vector<double> &vdot)
{
    if (vdot.size() != freedoms.size())
    {
        printf(
            "[error] Joint %d SetFreedomValueDot size %d != the dof num %d\n",
            this->mId, vdot.size(), GetNumOfFreedom());
        exit(1);
    }
    for (int i = 0; i < freedoms.size(); i++)
    {
        freedoms[i].vdot = vdot[i];
    }
}

/**
 * \brief           Get the freedom's velocity value vector "vdot"
 * \param vdot      the same as in GetFreedomValueDot
*/
void Joint::GetFreedomValueDot(std::vector<double> &vdot) const
{
    vdot.resize(freedoms.size());
    for (int i = 0; i < freedoms.size(); i++)
    {
        vdot[i] = freedoms[i].vdot;
    }
}

/**
 * \brief           Get the freedom velocity of this joint
 * 
 * For spherical joint, it's [xdot, ydot, zdot]
 * For revolute joint, it is [xdot]
 * For root joint, it 's [txdot, tydot, tzdot, xdot, ydot, zdot]
 * For fixed joint, 0 vector
*/
tVectorXd Joint::GetJointLocalVel() const
{
    std::vector<double> vel;
    GetFreedomValueDot(vel);
    tVectorXd res = tVectorXd::Zero(vel.size());
    for (int i = 0; i < vel.size(); i++)
        res[i] = vel[i];
    return res;
}

/**
 * \brief           Get the freedom offset of this joint in global freedom sequence
*/
int Joint::GetOffset() const
{
    if (freedoms.size() != 0)
    {

        // not fixed joint
        return freedoms[0].id;
    }
    else
    {
        if (dependent_dof_id.size() != 0)
        {
            // fixed joint, but not root
            return dependent_dof_id[dependent_dof_id.size() - 1];
        }
        else
        {
            // fixed root joint
            return 0;
        }
    }
}

/**
 * \brief           Get the local jkw of this joint
 * 
 * [w_local]    = dRdt * R^T
 *  w_local     = [dRdq * R^T] * qdot
 *              = Jw_local * qdot
*/
const tMatrixXd &Joint::GetLocalJkw() const { return this->JK_w_local; }

/**
 * \brief           Compute the local jkw of this joint
 * [w_local]    = dRdt * R^T
 *  w_local     = [dRdq * R^T] * qdot
 *              = Jw_local * qdot
*/
void Joint::ComputeLocalJkw()
{
    JK_w_local.setZero();
    // 1. get the local rotation
    tMatrix3d local_rotation = GetRotations();
    for (int i = 0; i < this->local_freedom; i++)
    {
        // 1. get the dRdq * RT

        tMatrix3d dRdq_RT =
            mTq[i].topLeftCorner<3, 3>() * local_rotation.transpose();

        // 2. check that it's a skew matrix
        // 3. extract the skew vector and put it into the matrix's column
        tVector col = btMathUtil::SkewMatToVector(dRdq_RT);
        JK_w_local.col(i) = col.segment(0, 3);
    }
}