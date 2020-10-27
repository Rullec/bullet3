#include "Link.h"
#include "Printer.h"
#include "tools.h"
//#define OLD_VERSION_JDOT

Link::Link(const BaseObjectParams &param) : BaseObject(param)
{
    link_omega.setZero();
    link_vel.setZero();
    col_group = 1;
}

Link::Link(const BaseObjectJsonParam &param) : BaseObject(param)
{
    ComputeIbody();
    link_omega.setZero();
    link_vel.setZero();
    col_group = 1;
}

void Link::Tell()
{
    std::cout << name /*<< ":\nprev_freedom: " << total_freedoms*/ << std::endl;
    std::cout << pos << std::endl;

    // std::cout << name << ":\nlocal_pos: " << local_pos[0] << ", " <<
    // local_pos[1] << ", " << local_pos[2] << "\n";
}

void Link::UpdateState(bool compute_gradient)
{
    // 1. compute orientation
    orientation.setIdentity();
    orientation = mesh_transform.matrix().cast<double>().topLeftCorner<3, 3>();
    // 2. compute local transform
    ComputeLocalTransform();
    // 3. compute global transform
    ComputeGlobalTransform();

    if (compute_gradient)
    {
        // // 4. update mwq for link
        UpdateMWQ();
        // // 5. compute jk
        ComputeJKv();
        ComputeJKw();
        ComputeJK();
        if (compute_second_derive)
        {
            tVector3d p(0, 0, 0);
            ComputeDJkvdq(p);
            ComputeDJkwdq();
        }
        // std::cout << name << ":\n";
        // std::cout << JK_w << std::endl;
        // std::cout << "====\n";
    }
}

void Link::UpdateMeshMatrix()
{
    mesh_matrix = global_transform.cast<float>() * scale_matrix;
}

void Link::UpdateMWQ()
{
    for (int i = 0; i < total_freedoms; ++i)
    {
        // mWq[i] = parent->GetMWQ(i) * local_transform;
        Tools::AVX4x4v1(parent_joint->GetMWQ(i), local_transform, mWq[i]);
    }
}

void Link::ComputeJKv_dot(tVectorXd &q_dot, tVector3d &p)
{
    assert(parent_joint != nullptr);
    // JK_v_dot.setZero();
#ifdef OLD_VERSION_JDOT
    for (int i = 0; i < total_freedoms; i++)
    {
        jkv_dq[i].setZero();
        for (int j = 0; j < total_freedoms; ++j)
        {
            const tVector column = parent_joint->GetMWQQ(i, j) *
                                   local_transform * Tools::GettVector(p);
            jkv_dq[i].col(prev_freedom_id[j]) = Tools::GettVector3d(column);
        }
        JK_v_dot += jkv_dq[i] * q_dot[i];
    }
#else
    // std::cout << GetId() << std::endl;
    for (int i = 0; i < 3; ++i)
    {
        // std::cout << jkv_dq[i].rows() << " " << jkv_dq[i].cols() <<
        // std::endl; std::cout << q_dot.size() << std::endl;
        JK_v_dot.row(i).noalias() = (jkv_dq[i] * q_dot).transpose();
    }
#endif
}

void Link::ComputeJKw_dot(tVectorXd &q_dot)
{
    assert(parent_joint != nullptr);
    // JK_w_dot.setZero();

#ifdef OLD_VERSION_JDOT
    for (int i = 0; i < total_freedoms; ++i)
    {
        jkw_dq[i].setZero();
        for (int j = 0; j < total_freedoms; ++j)
        {
            const tMatrix3d m =
                parent_joint->GetMWQQ(i, j).topLeftCorner<3, 3>() *
                    local_transform.topLeftCorner<3, 3>() *
                    global_transform.topLeftCorner<3, 3>().transpose() +
                mWq[j].topLeftCorner<3, 3>() *
                    mWq[i].topLeftCorner<3, 3>().transpose();
            const tVector3d column = Tools::FromSkewSymmetric(m);
            jkw_dq[i].col(prev_freedom_id[j]) = column;
        }
        JK_w_dot += jkw_dq[i] * q_dot[i];
    }
    // std::cout << JK_w_dot << std::endl;
    // std::cout << "===\n";
#else
    for (int i = 0; i < 3; ++i)
    {
        JK_w_dot.row(i).noalias() = (jkw_dq[i] * q_dot).transpose();
        // std::cout << "Jkw_dq " << i << " = \n"
        // 		  << jkw_dq[i] << std::endl;
    }
    // std::cout << "Jkw dot = \n"
    // 		  << JK_w_dot << std::endl;
#endif
}

/**
 * \brief           Given the local pos of a point attached at this link, calculate dJkv/dq
*/
void Link::ComputeDJkvdq(const tVector3d &p)
{
    assert(parent_joint != nullptr);
#ifndef OLD_VERSION_JDOT

    for (int i = 0; i < 3; i++)
    {
        jkv_dq[i].setZero();
    }
    tMatrix m_ = tMatrix::Zero();
    for (int i = 0; i < total_freedoms; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            // const tVector g = parent_joint->GetMWQQ(i, j) * local_transform *
            // Tools::GettVector(p);
            const tMatrix &m = parent_joint->GetMWQQ(i, j);
            Tools::AVX4x4v1(m, local_transform, m_);
            const tVector g = m_ * Tools::GettVector(p);
            jkv_dq[0].data()[dependent_dof_id[i] * global_freedom +
                             dependent_dof_id[j]] = g[0];
            jkv_dq[1].data()[dependent_dof_id[i] * global_freedom +
                             dependent_dof_id[j]] = g[1];
            jkv_dq[2].data()[dependent_dof_id[i] * global_freedom +
                             dependent_dof_id[j]] = g[2];

            jkv_dq[0].data()[dependent_dof_id[j] * global_freedom +
                             dependent_dof_id[i]] = g[0];
            jkv_dq[1].data()[dependent_dof_id[j] * global_freedom +
                             dependent_dof_id[i]] = g[1];
            jkv_dq[2].data()[dependent_dof_id[j] * global_freedom +
                             dependent_dof_id[i]] = g[2];
        }
    }
#endif
}

void Link::ComputeDJkwdq()
{
    assert(parent_joint != nullptr);
#ifndef OLD_VERSION_JDOT
    // for (int k = 0; k < 3; k++)
    // {
    // 	jkw_dq[k].setZero();
    // }
    tMatrix3d mij, gij, gji;
    tMatrix3d m;

    tMatrix3d local_trans_multiple_global_trans_T =
        local_transform.topLeftCorner<3, 3>() *
        global_transform.topLeftCorner<3, 3>().transpose();
    tEigenArr<tMatrix3d> mWq3x3_T(total_freedoms);
    for (int i = 0; i < total_freedoms; ++i)
        mWq3x3_T[i].noalias() = mWq[i].topLeftCorner<3, 3>().transpose();
    // i - row number
    // std::cout << "link " << this->GetId() << " total dof = " <<
    // total_freedoms << std::endl;
    for (int i = 0; i < total_freedoms; ++i)
    {
        int global_id_i = dependent_dof_id[i];
        int global_id_j = -1;
        const tMatrix3d &mWq_i_topleft = mWq[i].topLeftCorner<3, 3>();
        // j - col number
        for (int j = 0; j <= i; ++j)
        {
            global_id_j = dependent_dof_id[j];
            /*
                    dJ/dq \in R^{3xnxn}

                    J[0] = J.row(0) \in R^{nx1}

                    dJ[0]/dq \in R^{nxn}

                    dJ[0]_i/dq_j = jkw_dq[0](i, j)

                    J[0]_i = [dR/dqi * R^T]^{-1}[0] \in R

                    []^{-1} means a reverse operation of skew matrix. its result
               is 3x1 vector we do index it by [0] to access the first element
               of it. (x direction)

                    dJ[0]_i/dq_j = d ( [dR/dqi * R^T]^{-1}[0] ) dq_j
                                            = gij
            */
            m.noalias() = parent_joint->GetMWQQ(i, j).topLeftCorner<3, 3>() *
                          local_trans_multiple_global_trans_T; // 7.93
            // m.noalias() = parent_joint->GetMWQQ(i, j).topLeftCorner<3, 3>() *
            // 			  local_transform.topLeftCorner<3, 3>();

            // mij.noalias() = mWq_i_topleft * mWq[j].topLeftCorner<3,
            // 3>().transpose();  // 8.17
            mij.noalias() = mWq_i_topleft * mWq3x3_T[j]; // 8.17
            gij.noalias() = m + mij;
            // tVector3d g = Tools::FromSkewSymmetric(gij);
            // if (g.norm() > 1e-10)
            // {
            // 	std::cout << "[debug] g" << i << j << " = " << g.transpose() <<
            // std::endl; 	std::cout << "[debug] mat_g" << i << j << " = \n"
            // 			  << gij.transpose() << std::endl;
            // 	exit(0);
            // }
            // todo fix local freedom id
            // jkw_dq[0].data()[i * global_freedom + dependent_dof_id[j]] =
            // g[0]; jkw_dq[1].data()[i * global_freedom + dependent_dof_id[j]]
            // = g[1]; jkw_dq[2].data()[i * global_freedom +
            // dependent_dof_id[j]] = g[2]; gij.data()[5], [6], [1] is the 0, 1,
            // 2 value of it corosponding skew vector
            jkw_dq[0](global_id_i, global_id_j) = gij.data()[5];
            jkw_dq[1](global_id_i, global_id_j) = gij.data()[6];
            jkw_dq[2](global_id_i, global_id_j) = gij.data()[1];

            gji.noalias() = m + mij.transpose();
            // g = Tools::FromSkewSymmetric(gji);
            // if (g.norm() > 1e-10)
            // {
            // 	std::cout << "[debug] g" << j << i << " = " << g.transpose() <<
            // std::endl; 	std::cout << "[debug] mat_g" << j << i << " = \n"
            // 			  << gji.transpose() << std::endl;
            // 	exit(0);
            // }

            // jkw_dq[0].data()[dependent_dof_id[j] * global_freedom + i] =
            // g[0]; jkw_dq[1].data()[dependent_dof_id[j] * global_freedom + i]
            // = g[1]; jkw_dq[2].data()[dependent_dof_id[j] * global_freedom +
            // i] = g[2];

            // gji.data()[5], [6], [1] is the 0, 1, 2 value of it corosponding
            // skew vector
            jkw_dq[0](global_id_j, global_id_i) = gji.data()[5];
            jkw_dq[1](global_id_j, global_id_i) = gji.data()[6];
            jkw_dq[2](global_id_j, global_id_i) = gji.data()[1];
        }
    }
    // for (int i = 0; i < total_freedoms; i++)
    // {
    // 	std::cout << i << " " << dependent_dof_id[i] << std::endl;
    // }
    // exit(0);
#endif
}

tMatrix &Link::GetMWQQ(int i, int j)
{
    Printer::Error("Calling GetMWQQ in Link Object");
    assert(false);
    return mWq[i];
}

void Link::InitTerms()
{
    if (parent)
    {
        InitPrevFreedomIds();
        total_freedoms = parent->GetNumTotalFreedoms();
        parent_joint = parent;
    }
    else
    {
        std::cout << "[ERROR] link: " << name << "has no parent\n";
    }
    mass_matrix.resize(6, 6);
    mass_matrix.setZero();
    JK_v.resize(3, global_freedom);
    JK_v.setZero();
    JK_w.resize(3, global_freedom);
    JK_w.setZero();
    JK.resize(6, global_freedom);
    mWq.resize(total_freedoms, tMatrix::Zero());
    JK_v_dot.resize(3, global_freedom);
    JK_w_dot.resize(3, global_freedom);
    JK_dot.resize(6, global_freedom);
    JK_dot.setZero();
#ifdef OLD_VERSION_JDOT
    jkv_dq.resize(global_freedom, tMatrixXd::Zero(3, global_freedom));
    jkw_dq.resize(global_freedom, tMatrixXd::Zero(3, global_freedom));
#else
    // jkv_dq.resize(3, tMatrixXd::Zero(global_freedom, global_freedom));
    // jkw_dq.resize(3, tMatrixXd::Zero(global_freedom, global_freedom));
    jkv_dq.resize(3);
    jkw_dq.resize(3);
    for (int i = 0; i < 3; i++)
    {
        jkv_dq[i].noalias() = tMatrixXd::Zero(global_freedom, global_freedom);
        jkw_dq[i].noalias() = tMatrixXd::Zero(global_freedom, global_freedom);
    }

#endif
}

void Link::ComputeMassMatrix()
{
    mass_matrix.setZero();
    mass_matrix(0, 0) = mass;
    mass_matrix(1, 1) = mass;
    mass_matrix(2, 2) = mass;

    mass_matrix.block(3, 3, 3, 3).noalias() =
        global_transform.topLeftCorner<3, 3>() * Ibody *
        global_transform.topLeftCorner<3, 3>().transpose();
    // std::cout << "raw inertia = \n"
    // 		  << Ibody << std::endl;

    // std::cout << "R = \n"
    // 		  << global_transform.topLeftCorner<3, 3>() << std::endl;

    // std::cout << "final inertia = \n"
    // 		  << mass_matrix << std::endl;
}

void Link::SetLinkVel(const tVector3d &link_vel_)
{
    this->link_vel = link_vel_;
}
void Link::SetLinkOmega(const tVector3d &link_omega_)
{
    this->link_omega = link_omega_;
}
tVector3d Link::GetLinkVel() const { return link_vel; }
tVector3d Link::GetLinkOmega() const { return link_omega; }

/**
 * \brief					Get the max length of a link's body
 * diagnoal
 *
 */
double Link::GetLinkMaxLength() const
{
    std::cout << "link get max length exit\n";
    exit(0);
    return 0;
}

/**
 * \brief					Get the collision group
 * 			collision group 0: disable collision within the
 * multibody colision group 1: enable collision within the multibody
 */
int Link::GetColGroup() const { return col_group; }
void Link::SetColGroup(int flag)
{
    col_group = flag;
    if (col_group != 0 && col_group != 1)
    {
        std::cout << "[error] Invalid link colgroup " << col_group << std::endl;
        exit(0);
    }
}

/**
 * \brief					this function tries to calculate
 * :
 *
 * 			\frac{dJkv}{dq_{target_dof_id}} \in R^{3 \times n}
 *
 * 			Jkv \in R^{3 \times n}
 * 			target_dof_id \in [0, n-1]
 */
tMatrixXd Link::GetTotalDofdJKv_dq(int target_dof_id) const
{
    // // 1. check whether the target_dof_id effect the Jkv
    tMatrixXd dJkv_dq = tMatrixXd::Zero(3, global_freedom);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < global_freedom; j++)
        {
            dJkv_dq(i, j) = GetJKv_dq(i)(j, target_dof_id);
        }
    return dJkv_dq;
}

tMatrixXd Link::GetTotalDofdJKw_dq(int target_dof_id) const
{
    tMatrixXd dJkw_dq = tMatrixXd::Zero(3, global_freedom);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < global_freedom; j++)
        {
            dJkw_dq(i, j) = GetJKw_dq(i)(j, target_dof_id);
        }
    return dJkw_dq;
}
/**
 * \brief           Compute dM/dq, M is the mass matrix
 * M =  [ I3m  0 ]
 *      [ 0   RI0RT]
 * 
 * dMdq = [ 0  0 ]
 *      [ 0   d(RI0RT)/dq]
 * dRI0RTdq = dRdq I0RT + R*I0d(RT)dq
*/
void Link::ComputedMdq(tEigenArr<tMatrixXd> &dMdq)
{
    // std::cout << "[log] link " << GetId() << " dmdq begin\n";
    int dMdq_shape = 6;
    dMdq.resize(global_freedom, tMatrixXd::Zero(dMdq_shape, dMdq_shape));
    tMatrix3d R = global_transform.topLeftCorner<3, 3>();
    // 1. shape the dRdq
    tEigenArr<tMatrix3d> dRdq(global_freedom, tMatrix3d::Zero());
    for (int local_dof_id = 0; local_dof_id < total_freedoms; local_dof_id++)
    {
        auto parent_joint = this->GetParent();
        // std::cout << "[log] joint " << parent_joint->GetId() << std::endl;

        int global_dof_id = GetPrevFreedomIds()[local_dof_id];

        dRdq[global_dof_id] = GetMWQ(local_dof_id).topLeftCorner<3, 3>();
    }

    // 2. shape the final result
    for (int global_dof_id = 0; global_dof_id < global_freedom; global_dof_id++)
    {
        dMdq[global_dof_id].block(3, 3, 3, 3) =
            dRdq[global_dof_id] * Ibody * R.transpose() +
            R * Ibody * dRdq[global_dof_id].transpose();
    }
}