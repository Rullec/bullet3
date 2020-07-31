#include "Joint.h"
#include "Printer.h"
#include "EulerAngelRotationMatrix.h"
// #include "BasePoint.h"
#include "tools.h"

Joint::Joint(BaseObjectParams& param) : BaseObject(param) {}

Joint::Joint(BaseObjectJsonParam& param) : BaseObject(param)
{
}

void Joint::Tell()
{
	std::cout << name /*<< ":\nprev_freedom: " << total_freedoms*/ << std::endl;
	std::cout << pos << std::endl;
	//std::cout << "parent_joint: " << parent_joint->GetName() << std::endl;
}

Freedom* Joint::AddFreedom(Freedom& f)
{
	Freedom fs(f);
	freedoms.push_back(fs);
	local_freedom = static_cast<int>(freedoms.size());
	return &freedoms[freedoms.size() - 1];
}

Freedom* Joint::GetFreedoms(int order)
{
	assert(order < freedoms.size() && order >= 0);
	return &freedoms[order];
}

Freedom* Joint::GetFreedomByAxis(tVector3d axis, int type)
{
	if (freedoms.empty()) return nullptr;
	for (auto& f : freedoms)
	{
		if (f.axis == axis && f.type == type)
		{
			return &f;
		}
	}
	return nullptr;
}

/**
 * \brief					��ʼ����ǰ�ؽ����������ɶȸ���, mTq �� mWq.
 */
void Joint::InitTerms()
{
	if (parent)
	{
		InitPrevFreedomIds();
		prev_freedoms = parent->GetNumTotalFreedoms();
		if (!parent_joint) parent_joint = parent->GetParent();
		const tMatrix& m_neg_ini = parent_joint->GetNegInitRotationtMatrix();
		//rot_parent_child = m_neg_ini * init_rotation_matrix_4x4;
		Tools::AVX4x4v1(m_neg_ini, init_rotation_matrix_4x4, rot_parent_child);
		local_pos_parent_joint = Tools::GettVector3d(parent->GetLocalTransform() * Tools::GettVector(local_pos));
		//local_pos_parent_joint = local_pos *2;
		// before 2020/04/08 local pos is from this joint to parent joint
		// but now we modify the kinematics chain, local pos will be set to the parent joint
	}
	else
	{
		rot_parent_child.setIdentity();
		local_pos_parent_joint = local_pos;
	}
	for (auto& f : freedoms)
	{
		dependent_dof_id.push_back(f.id);
	}
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
	mTq.resize(local_freedom);
	mWq.resize(total_freedoms);

	JK_w.resize(3, global_freedom);
	JK_v.resize(3, global_freedom);
	JK.resize(6, global_freedom);
	JK_w.setZero();
	JK_v.setZero();
	JK.setZero();

	JK.block(3, 0, 3, global_freedom);

	//JK_v_dot.resize(3, global_freedom);
	//JK_w_dot.resize(3, global_freedom);
	//JK_dot.resize(6 , global_freedom);
	//JK_dot.setZero();

	//jkv_dq.resize(global_freedom, tMatrixXd::Zero(3, global_freedom));
	//jkw_dq.resize(global_freedom, tMatrixXd::Zero(3, global_freedom));

	mTqq.resize(local_freedom);

	for (int i = 0; i < local_freedom; ++i)
	{
		mTqq[i].resize(i + 1);
	}

	mWqq.resize(total_freedoms);
	for (int i = 0; i < total_freedoms; i++)
	{
		mWqq[i].resize(i + 1);
	}
}

/**
 * \brief					���µ�ǰ�Ƕ���joint��ȫ����Ϣ
 * \param compute_gradient	�Ƿ�����ݶȣ��ڲ���ʱ�������ݶȣ����Ż�ʱ��Ҫ�����ݶȡ�?
 */
void Joint::UpdateState(bool compute_gradient)
{
	// 1. compute orientation
	GetRotations(orientation);
	// 2. compute local transform
	ComputeLocalTransform();
	// 3. compute global transform
	ComputeGlobalTransform();
	// 4. compute mTq, mWq
	if (compute_gradient)
	{
		ComputeTransformFirstDerive();
		ComputeGlobalTransformFirstDerive();

		ComputeJKv();
		ComputeJKw();
		ComputeJK();
		if (compute_second_derive)
		{
			ComputeLocalSecondDeriveMatrix();
			ComputeLocalTransformSecondDerive();
			ComputeGlobalTransformSecondDerive();
		}
	}
}

/**
 * \brief					���µ�ǰ���󣬼�����ת����
 */
void Joint::UpdateMatrix()
{
	int count = 0;
	for (auto& f : freedoms)
	{
		if (f.axis[0] == 1)
		{
			xconventionTransform(r_m[count], f.v);
			//xconventionRotation_dx(r_m_first_deriv[count], f.v);
		}
		else if (f.axis[1] == 1)
		{
			yconventionTransform(r_m[count], f.v);
			//yconventionRotation_dy(r_m_first_deriv[count], f.v);
		}
		else if (f.axis[2] == 1)
		{
			zconventionTransform(r_m[count], f.v);
			//zconventionRotation_dz(r_m_first_deriv[count], f.v);
		}
		count++;
	}
}

/**
 * \brief					����ֲ������һ�׵�������
 *							\frac{\partial R_L}{\partial q}
 */
void Joint::ComputeLocalFirstDeriveMatrix()
{
	int count = 0;
	for (auto& f : freedoms)
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
 * \brief					����ֲ�����Ķ��׵�������
 *							\frac{\partial^2 R_L}{\partial q_i \partial q_j}
 */
void Joint::ComputeLocalSecondDeriveMatrix()
{
	int count = 0;
	for (auto& f : freedoms)
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
 * \brief					���� local orientation
 * \param m					���ؽ��?
 */
void Joint::GetRotations(tMatrix3d& m)
{
	m.setIdentity();
	UpdateMatrix();
	for (auto& rotation_matrix : r_m)
	{
		m = rotation_matrix.topLeftCorner<3, 3>() * m;
	}
}

/**
 * \brief					���� mTq
 */
void Joint::ComputeTransformFirstDerive()
{
	ComputeLocalFirstDeriveMatrix();
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
		//mTq[0] = r_m[2] * r_m[1] * r_m_first_deriv[0];
		//mTq[1] = r_m[2] * r_m_first_deriv[1] * r_m[0];
		//mTq[2] = r_m_first_deriv[2] * r_m[1] * r_m[0];

		Tools::AVX4x4v1_3mat(r_m[2], r_m[1], r_m_first_deriv[0], mTq[0]);
		Tools::AVX4x4v1_3mat(r_m[2], r_m_first_deriv[1], r_m[0], mTq[1]);
		Tools::AVX4x4v1_3mat(r_m_first_deriv[2], r_m[1], r_m[0], mTq[2]);
	}
}

/**
 * \brief					ͨ�� mTq ���� mWq
 */
void Joint::ComputeGlobalTransformFirstDerive()
{
	for (int i = 0; i < prev_freedoms; i++)
	{  //for root joint prev_freedom is 0, so this part will not be executed
		if (parent_joint)
		{
			const tMatrix& parent_mwqi = parent_joint->GetMWQ(i);
			//mWq[i] = parent_joint->GetMWQ(i) * rot_parent_child * local_transform;
			Tools::AVX4x4v1_3mat(parent_mwqi, rot_parent_child, local_transform, mWq[i]);
		}
	}
	for (int i = 0; i < local_freedom; i++)
	{
		if (parent_joint)
		{
			const tMatrix& m = parent_joint->GetGlobalTransform();
			//mWq[i + prev_freedoms] = m * rot_parent_child * mTq[i];
			Tools::AVX4x4v1_3mat(m, rot_parent_child, mTq[i], mWq[i + prev_freedoms]);
		}
		else
		{
			//mWq[i + prev_freedoms] = init_rotation_matrix_4x4 * mTq[i];
			Tools::AVX4x4v1(init_rotation_matrix_4x4, mTq[i], mWq[i + prev_freedoms]);
		}
	}
}

/**
 * \brief					���� mTqq
 *							mTqq = \frac{\partial^2 R_L}{\partial q_i \partial q_j}
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
					//mTqq[i][j] = r_m_first_deriv[k] * mTqq[i][j];
					Tools::AVX4x4v1(r_m_first_deriv[k], mTqq[i][j], t);
					mTqq[i][j] = t;
				}
				else if (k == i && i == j)
				{
					//mTqq[i][j] = r_m_second_deriv[k] * mTqq[i][j];
					Tools::AVX4x4v1(r_m_second_deriv[k], mTqq[i][j], t);
					mTqq[i][j] = t;
				}
				else if (k != i && k == j)
				{
					//mTqq[i][j] = r_m_first_deriv[k] * mTqq[i][j];
					Tools::AVX4x4v1(r_m_first_deriv[k], mTqq[i][j], t);
					mTqq[i][j] = t;
				}
				else
				{
					//mTqq[i][j] = r_m[k] * mTqq[i][j];
					Tools::AVX4x4v1(r_m[k], mTqq[i][j], t);
					mTqq[i][j] = t;
				}
			}
		}
	}
}

/**
 * \brief					���� mWqq
 *							mWqq = \frac{\partial^2 R_W}{\partial q_i \partial q_j}
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
			//mWqq[i][j].noalias() = parent_joint->GetMWQQ(i, j) * parent_t_child * local_transform;
			Tools::AVX4x4v1_3mat(parent_joint->GetMWQQ(i, j), parent_t_child, local_transform, mWqq[i][j]);
		}
		//for (int j = 0; j < local_freedom; j++)	{
		//	if (parent_joint)
		//		mWqq[i][j + num_parent_dofs] = parent_joint->GetMWQ(i) * parent_t_child * mTq[j];
		//	else
		//		mWqq[i][j + num_parent_dofs] = init_rotation_matrix_4x4 * mTq[j];
		//}
	}

	for (int i = 0; i < local_freedom; i++)
	{
		for (int j = 0; j < num_parent_dofs; j++)
		{
			if (parent_joint)
			{
				//mWqq[num_parent_dofs + i][j].noalias() = parent_joint->GetMWQ(j) * parent_t_child * mTq[i];
				const tMatrix& m = parent_joint->GetMWQ(j);
				Tools::AVX4x4v1_3mat(m, parent_t_child, mTq[i], mWqq[num_parent_dofs + i][j]);
			}
			else
			{
				//mWqq[num_parent_dofs + i][j].noalias() = init_rotation_matrix_4x4 * mTq[i];
				Tools::AVX4x4v1(init_rotation_matrix_4x4, mTq[i], mWqq[num_parent_dofs + i][j]);
			}
		}

		for (int j = 0; j <= i; j++)
		{
			if (parent_joint)
			{
				//mWqq[num_parent_dofs + i][num_parent_dofs + j].noalias() = parent_joint->GetGlobalTransform() * rot_parent_child * mTqq[i][j];
				const tMatrix& m = parent_joint->GetGlobalTransform();
				Tools::AVX4x4v1_3mat(m, rot_parent_child, mTqq[i][j], mWqq[num_parent_dofs + i][num_parent_dofs + j]);
			}
			else
			{
				//mWqq[num_parent_dofs + i][num_parent_dofs + j].noalias() = init_rotation_matrix_4x4 * mTqq[i][j];
				Tools::AVX4x4v1(init_rotation_matrix_4x4, mTqq[i][j], mWqq[num_parent_dofs + i][num_parent_dofs + j]);
			}
		}
	}
}

/**
 * \brief					����Ŀ���p, ����������������ɶȵ�jacobian����
 * \param j					�����õ���jacobian����
 * \param p					�������Ŀ���
 */
void Joint::ComputeJacobiByGivenPoint(tMatrixXd& j, tVector& p)
{
	if (j.rows() != 3 || j.cols() != total_freedoms) j.resize(3, total_freedoms);

	tVector column;
	for (size_t i = 0; i < mWq.size(); i++)
	{
		Tools::MatMul4x1(mWq[i], p, column);
		// j.col(i) = Tools::GettVector3d(column);
		j.col(i).noalias() = column.segment(0, 3);
	}
}

void Joint::ComputeHessianByGivenPoint(EIGEN_V_MATXD& ms, tVector& p)
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
void Joint::ComputeJacobiByGivenPointTotalDOF(tMatrixXd& j, tVector p)
{
	// 1. get the dependent Jacobian
	ComputeJacobiByGivenPoint(local_jac_buf, p);
	// std::cout <<"Joint::Compute short = \n" << j_dependent_dof << std::endl;
	assert(local_jac_buf.cols() == this->total_freedoms);

	// 2. map this jacobian to global jacobian
	if (j.cols() != 3 || j.rows() != global_freedom) j.resize(3, global_freedom);
	j.setZero();
	// std::cout <<"prev size = " << prev_freedoms <<", freedoms size = " << freedoms.size() << std::endl;

	/* 	Attention: the size of prev_freedoms_id should be equal to prev_freedoms
		But for root joint, this law is falsely broken. It does not effect other behavior but just a bug.
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

//void Joint::ComputeJK_dot() {
//	JK_dot.block(0, 0, 3, global_freedom) = JK_v_dot;
//	JK_dot.block(3, 0, 3, global_freedom) = JK_w_dot;
//}

void Joint::ComputeJKv_dot(tVectorXd& q_dot, tVector3d& p)
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

void Joint::ComputeJKw_dot(tVectorXd& q_dot)
{
	Printer::Error("Calling ComputeJKw_dot in Joint");
	exit(-1);
	JK_w_dot.setZero();
	for (size_t i = 0; i < mWqq.size(); ++i)
	{
		jkw_dq[i].setZero();
		for (int j = 0; j < total_freedoms; ++j)
		{
			const tMatrix3d m = mWqq[i][j].topLeftCorner<3, 3>() * global_transform.topLeftCorner<3, 3>().transpose() + mWq[j].topLeftCorner<3, 3>() * mWq[i].topLeftCorner<3, 3>().transpose();
			const tVector3d column = Tools::FromSkewSymmetric(m);
			jkw_dq[i].col(dependent_dof_id[j]) = column;
		}
		JK_w_dot += jkw_dq[i] * q_dot[dependent_dof_id[i]];
	}
}

void Joint::ComputeLocalTransform()
{
	local_transform.topLeftCorner<3, 3>() = orientation;
	local_transform.data()[12] = local_pos_parent_joint.data()[0];  // *2;
	local_transform.data()[13] = local_pos_parent_joint.data()[1];  // *2;
	local_transform.data()[14] = local_pos_parent_joint.data()[2];  // *2;
	local_transform.data()[15] = 1.;
}

/**
 * \brief					���� global_transform ����
 */
void Joint::ComputeGlobalTransform()
{
	if (parent_joint)
	{
		const tMatrix& m = parent_joint->GetGlobalTransform();
		//global_transform = m * rot_parent_child * local_transform;
		Tools::AVX4x4v1_3mat(m, rot_parent_child, local_transform, global_transform);
	}
	else
	{
		//global_transform = init_rotation_matrix_4x4 * local_transform;
		Tools::AVX4x4v1(init_rotation_matrix_4x4, local_transform, global_transform);
	}
	tVector p(0, 0, 0, 1);
	tVector p_;
	Tools::MatMul4x1(global_transform, p, p_);
	pos[0] = p_[0];
	pos[1] = p_[1];
	pos[2] = p_[2];
}

tMatrix& Joint::GetMWQQ(int i, int j)
{
	int r = std::max(i, j);
	int c = std::min(i, j);
	return mWqq[r][c];
}

const tMatrixXd& Joint::GetJKDot() const
{
	Printer::Error("Calling GetJKDot in Joint");
	exit(-1);
	return mWqq[0][0];
}

int Joint::GetNumOfFreedom()
{
	return static_cast<int>(freedoms.size());
}

void Joint::SetFreedomValue(int id, double v)
{
	freedoms[id].v = v;
}

void Joint::GetFreedomValue(int id, double& v)
{
	v = freedoms[id].v;
}

void Joint::SetFreedomValue(std::vector<double>& v)
{
	for (auto& f : freedoms)
	{
		f.v = v[f.id];
	}
}

void Joint::GetFreedomValue(std::vector<double>& v)
{
	for (auto& f : freedoms)
	{
		v[f.id] = f.v;
	}
}

void Joint::CleanGradient()
{
	for (auto& f : freedoms)
	{
		f.clear_grad();
	}
}
