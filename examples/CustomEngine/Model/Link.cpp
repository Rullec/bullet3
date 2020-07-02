#include "Link.h"
#include "tools.h"
#include "Printer.h"
//#define OLD_VERSION_JDOT

Link::Link(BaseObjectParams& param) : BaseObject(param)
{
}

Link::Link(BaseObjectJsonParam& param) : BaseObject(param)
{
	ComputeIbody();
}

void Link::Tell()
{
	std::cout << name /*<< ":\nprev_freedom: " << total_freedoms*/ << std::endl;
	std::cout << pos << std::endl;

	//std::cout << name << ":\nlocal_pos: " << local_pos[0] << ", " << local_pos[1] << ", " << local_pos[2] << "\n";
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
		//	// 4. update mwq for link
		UpdateMWQ();
		//	// 5. compute jk
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
		//mWq[i] = parent->GetMWQ(i) * local_transform;
		Tools::AVX4x4v1(parent_joint->GetMWQ(i), local_transform, mWq[i]);
	}
}

void Link::ComputeJKv_dot(tVectorXd& q_dot, tVector3d& p)
{
	assert(parent_joint != nullptr);
	JK_v_dot.setZero();
#ifdef OLD_VERSION_JDOT
	for (int i = 0; i < total_freedoms; i++)
	{
		jkv_dq[i].setZero();
		for (int j = 0; j < total_freedoms; ++j)
		{
			const tVector column = parent_joint->GetMWQQ(i, j) * local_transform * Tools::GettVector(p);
			jkv_dq[i].col(prev_freedom_id[j]) = Tools::GettVector3d(column);
		}
		JK_v_dot += jkv_dq[i] * q_dot[i];
	}
#else
	for (int i = 0; i < 3; ++i)
	{
		JK_v_dot.row(i) = (jkv_dq[i] * q_dot).transpose();
	}
#endif
}

void Link::ComputeJKw_dot(tVectorXd& q_dot)
{
	assert(parent_joint != nullptr);
	JK_w_dot.setZero();

#ifdef OLD_VERSION_JDOT
	for (int i = 0; i < total_freedoms; ++i)
	{
		jkw_dq[i].setZero();
		for (int j = 0; j < total_freedoms; ++j)
		{
			const tMatrix3d m = parent_joint->GetMWQQ(i, j).topLeftCorner<3, 3>() * local_transform.topLeftCorner<3, 3>() * global_transform.topLeftCorner<3, 3>().transpose() + mWq[j].topLeftCorner<3, 3>() * mWq[i].topLeftCorner<3, 3>().transpose();
			const tVector3d column = Tools::FromSkewSymmetric(m);
			jkw_dq[i].col(prev_freedom_id[j]) = column;
		}
		JK_w_dot += jkw_dq[i] * q_dot[i];
	}
	//std::cout << JK_w_dot << std::endl;
	//std::cout << "===\n";
#else
	for (int i = 0; i < 3; ++i)
	{
		JK_w_dot.row(i) = (jkw_dq[i] * q_dot).transpose();
	}
#endif
}

void Link::ComputeDJkvdq(tVector3d& p)
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
			//const tVector g = parent_joint->GetMWQQ(i, j) * local_transform * Tools::GettVector(p);
			const tMatrix& m = parent_joint->GetMWQQ(i, j);
			Tools::AVX4x4v1(m, local_transform, m_);
			const tVector g = m_ * Tools::GettVector(p);
			jkv_dq[0].data()[i * global_freedom + dependent_dof_id[j]] = g[0];
			jkv_dq[1].data()[i * global_freedom + dependent_dof_id[j]] = g[1];
			jkv_dq[2].data()[i * global_freedom + dependent_dof_id[j]] = g[2];

			jkv_dq[0].data()[dependent_dof_id[j] * global_freedom + i] = g[0];
			jkv_dq[1].data()[dependent_dof_id[j] * global_freedom + i] = g[1];
			jkv_dq[2].data()[dependent_dof_id[j] * global_freedom + i] = g[2];
		}
	}
#endif
}

void Link::ComputeDJkwdq()
{
	assert(parent_joint != nullptr);
#ifndef OLD_VERSION_JDOT
	for (int i = 0; i < 3; i++)
	{
		jkw_dq[i].setZero();
	}
	tMatrix3d mji, gji, gij;
	tMatrix3d m;
	for (int i = 0; i < total_freedoms; ++i)
	{
		for (int j = 0; j <= i; ++j)
		{
			m.noalias() = parent_joint->GetMWQQ(i, j).topLeftCorner<3, 3>() *
						  local_transform.topLeftCorner<3, 3>() *
						  global_transform.topLeftCorner<3, 3>().transpose();

			mji = mWq[j].topLeftCorner<3, 3>() * mWq[i].topLeftCorner<3, 3>().transpose();
			gji = m + mji;
			tVector3d g = Tools::FromSkewSymmetric(gji);
			// todo fix local freedom id
			jkw_dq[0].data()[i * global_freedom + dependent_dof_id[j]] = g[0];
			jkw_dq[1].data()[i * global_freedom + dependent_dof_id[j]] = g[1];
			jkw_dq[2].data()[i * global_freedom + dependent_dof_id[j]] = g[2];

			gij = m + mji.transpose();
			g = Tools::FromSkewSymmetric(gij);
			jkw_dq[0].data()[dependent_dof_id[j] * global_freedom + i] = g[0];
			jkw_dq[1].data()[dependent_dof_id[j] * global_freedom + i] = g[1];
			jkw_dq[2].data()[dependent_dof_id[j] * global_freedom + i] = g[2];
		}
	}
#endif
}

tMatrix& Link::GetMWQQ(int i, int j)
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
	jkv_dq.resize(3, tMatrixXd::Zero(global_freedom, global_freedom));
	jkw_dq.resize(3, tMatrixXd::Zero(global_freedom, global_freedom));
#endif
}

void Link::ComputeMassMatrix()
{
	mass_matrix.setZero();
	mass_matrix(0, 0) = mass;
	mass_matrix(1, 1) = mass;
	mass_matrix(2, 2) = mass;

	mass_matrix.block(3, 3, 3, 3) = global_transform.topLeftCorner<3, 3>() * Ibody * global_transform.topLeftCorner<3, 3>().transpose();
	// std::cout <<"Ibody = \n" << Ibody << std::endl;
	// std::cout <<"mass mat = \n" << mass_matrix << std::endl;
}
