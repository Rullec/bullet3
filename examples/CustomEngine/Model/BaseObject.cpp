#include "BaseObject.h"
#include "EulerAngelRotationMatrix.h"
#include "tools.h"
#include <iostream>

int BaseObject::GetNumOfChildren() const
{
	return static_cast<int>(children.size());
}

void BaseObject::SetParent(BaseObject* parent)
{
	BaseObject::parent = parent;
}

void BaseObject::SetLocalPos(float x, float y, float z)
{
	local_pos[0] = x;
	local_pos[1] = y;
	local_pos[2] = z;
}

void BaseObject::SetLocalPos(const tVector3d& local_pos)
{
	this->local_pos = local_pos;
}

int BaseObject::GetNumOfFreedom()
{
	return -1;
}

BaseObject* BaseObject::GetChild(int order)
{
	try
	{
		return children.at(order);
	}
	catch (std::out_of_range oor)
	{
		std::cout << name << " exception out of error\n";
	}
	return nullptr;
}

void BaseObject::SetPos(const tVector3d& pos)
{
	this->pos = pos;
}

void BaseObject::AddChild(BaseObject* ch)
{
	children.push_back(ch);
}

void BaseObject::ClearChildren()
{
	children.clear();
}

BaseObject* BaseObject::GetParent() const
{
	return parent;
}

void BaseObject::Tell()
{
	std::cout << name << ":\nlocal_pos: " << local_pos[0] << ", " << local_pos[1] << ", " << local_pos[2] << "\n";
}

void BaseObject::InitPrevFreedomIds()
{
	auto& parent_prev_ids = parent->GetPrevFreedomIds();
	for (size_t i = 0; i < parent_prev_ids.size(); i++)
	{
		dependent_dof_id.push_back(parent_prev_ids[i]);
	}
}

void BaseObject::SetLocalRot(const tVector3d& local_rot)
{
	init_rotation = local_rot;
	init_rotation_matrix.setIdentity();
	init_rotation_matrix_4x4.setIdentity();
	init_rotation_matrix = zconventionRotation(init_rotation[2]) *
						   yconventionRotation(init_rotation[1]) *
						   xconventionRotation(init_rotation[0]);

	init_rotation_matrix_4x4.topLeftCorner<3, 3>() = init_rotation_matrix;
}

void BaseObject::UpdateShape(BaseObjectShapeParam& param)
{
	shape_type = param.shape_type;
	if (param.width != 0) mesh_scale(0) = param.width;
	if (param.length != 0) mesh_scale(2) = param.length;
	if (param.height != 0) mesh_scale(1) = param.height;
	scale_matrix = Eigen::scale(mesh_scale.x(), mesh_scale.y(), mesh_scale.z());

	// only compute inertia for links
	if (!IsJoint()) ComputeIbody();
}

void BaseObject::ComputeIbody()
{
	Ibody.setZero();

	if (shape_type == ShapeType::CYLINDER)
	{
		double h = mesh_scale(1);
		double R = mesh_scale(0) / 2.;
		double M = mass;

		double Ixx = M * pow(h, 2) / 12.0 + M * pow(R, 2) / 4.;
		double Iyy = M * pow(R, 2) / 2.;
		double Izz = Ixx;

		Ibody(0, 0) = Ixx;
		Ibody(1, 1) = Iyy;
		Ibody(2, 2) = Izz;
	}
	else if (shape_type == ShapeType::BOX_SHAPE)
	{
		double width = mesh_scale(0);
		double height = mesh_scale(1);
		double length = mesh_scale(2);

		Ibody(0, 0) = mass * (pow(length, 2) + pow(height, 2)) / 12.;
		Ibody(1, 1) = mass * (pow(width, 2) + pow(length, 2)) / 12.;
		Ibody(2, 2) = mass * (pow(width, 2) + pow(height, 2)) / 12.;
	}
	else if (shape_type == ShapeType::SPHERE_SHAPE)
	{
		double radius = mesh_scale(0);
		Ibody = tMatrix3d::Identity() * 2.0 / 3 * mass * pow(radius, 2);
	}
	else
	{
		std::cout << "Unsupported type: " << shape_type << " in ComputeIbody\n";
		exit(1);
	}

	// std::cout << "Inertia: " << std::endl;
	// std::cout << name << std::endl;
	// std::cout << Ibody << std::endl;
	// std::cout << "===" << std::endl;
	// exit(0);
}

void BaseObject::ComputeLocalTransform()
{
	local_transform.topLeftCorner<3, 3>() = orientation;
	local_transform.data()[12] = local_pos.data()[0];
	local_transform.data()[13] = local_pos.data()[1];
	local_transform.data()[14] = local_pos.data()[2];
	local_transform.data()[15] = 1.;
}

void BaseObject::ComputeGlobalTransform()
{
	if (parent)
	{
		const tMatrix& m = parent->GetGlobalTransform();
		//global_transform = m * local_transform;
		Tools::AVX4x4v1(m, local_transform, global_transform);
	}
	else
	{
		global_transform = local_transform;
	}
	tVector p = global_transform * tVector(0, 0, 0, 1);
	pos[0] = p[0];
	pos[1] = p[1];
	pos[2] = p[2];
}

void BaseObject::ComputeJKw()
{
	JK_w.setZero();
	for (int i = 0; i < total_freedoms; ++i)
	{
		tMatrix3d w_skesym = mWq[i].topLeftCorner<3, 3>() * global_transform.topLeftCorner<3, 3>().transpose();
		JK_w.col(dependent_dof_id[i]) = Tools::FromSkewSymmetric(w_skesym);
	}
}

void BaseObject::ComputeJKv()
{
	JK_v.setZero();
	const tVector& p = tVector(0, 0, 0, 1);
	tVector r;
	for (int i = 0; i < total_freedoms; ++i)
	{
		Tools::MatMul4x1(mWq[i], p, r);
		JK_v.col(dependent_dof_id[i]) = Tools::GettVector3d(r);
	}
}

void BaseObject::ComputeJK()
{
	JK.block(0, 0, 3, global_freedom) = JK_v;
	JK.block(3, 0, 3, global_freedom) = JK_w;
}

void BaseObject::ComputeJK_dot()
{
	JK_dot.block(0, 0, 3, global_freedom) = JK_v_dot;
	JK_dot.block(3, 0, 3, global_freedom) = JK_w_dot;
}

BaseObject* BaseObject::GetFirstChild()
{
	if (children.empty())
		return nullptr;
	return children[0];
}

BaseObject::BaseObject(BaseObjectParams& param) : id(0),
												  parent_id(-1),
												  name(param.name),
												  local_pos(param.local_pos),
												  parent(nullptr),
												  mesh_scale(param.mesh_scale),
												  mesh_rotation(param.mesh_rotation),
												  mesh(param.mesh),
												  render_struct(nullptr),
												  mass(param.mass),
												  parent_joint(nullptr),
												  total_freedoms(-1),
												  prev_freedoms(0),
												  local_freedom(0),
												  init_rotation(param.local_rotation),
												  joint_type(JointType::INVALID_JOINT),
												  shape_type(-1),
												  global_freedom(0),
												  compute_second_derive(false)
{
	scale_matrix = Eigen::scale(mesh_scale.x(), mesh_scale.y(), mesh_scale.z());

	mesh_transform = Eigen::AngleAxisf(mesh_rotation(2), tVector3f::UnitZ()) *
					 Eigen::AngleAxisf(mesh_rotation(1), tVector3f::UnitY()) *
					 Eigen::AngleAxisf(mesh_rotation(0), tVector3f::UnitX()).toRotationMatrix();

	init_rotation_matrix.setIdentity();
	init_rotation_matrix_4x4.setIdentity();
	init_rotation_matrix = zconventionRotation(init_rotation[2]) *
						   yconventionRotation(init_rotation[1]) *
						   xconventionRotation(init_rotation[0]);
	init_rotation_matrix_4x4.topLeftCorner<3, 3>() = init_rotation_matrix;

	neg_init_rotation_matrix = xconventionRotation(-init_rotation[0]) *
							   yconventionRotation(-init_rotation[1]) *
							   zconventionRotation(-init_rotation[2]);

	neg_init_rotation_matrix_4x4.setIdentity();
	neg_init_rotation_matrix_4x4.topLeftCorner<3, 3>() = neg_init_rotation_matrix;

	model_matrix = tMatrix::Identity();
	mesh_matrix = model_matrix.cast<float>() * mesh_transform * scale_matrix;

	local_transform.setIdentity();
	local_transform.topLeftCorner<3, 3>() = init_rotation_matrix;
	local_transform.data()[12] = local_pos[0];
	local_transform.data()[13] = local_pos[1];
	local_transform.data()[14] = local_pos[2];
	local_transform.data()[15] = 1.0;
}

BaseObject::BaseObject(BaseObjectJsonParam& param) : id(param.id),
													 parent_id(param.parent_id),
													 name(param.name),
													 local_pos(param.local_pos),
													 parent(nullptr),
													 mesh_scale(param.mesh_scale),
													 mesh_rotation(param.mesh_rot),
													 mesh(param.mesh),
													 render_struct(nullptr),
													 mass(param.mass),
													 parent_joint(nullptr),
													 total_freedoms(-1),
													 prev_freedoms(0),
													 local_freedom(0),
													 init_rotation(param.local_rot),
													 joint_type(JointType::INVALID_JOINT),
													 Ibody(param.inertia),
													 shape_type(param.shape_type),
													 global_freedom(0),
													 compute_second_derive(false)
{
	assert(Ibody.norm() < 1e2);
	if (param.type == LINK)
	{
		init_rotation = tVector3d(0, 0, 0);
	}
	InitObject();
}

void BaseObject::InitObject()
{
	scale_matrix = Eigen::scale(mesh_scale.x(), mesh_scale.y(), mesh_scale.z());
	mesh_transform = Eigen::AngleAxisf(mesh_rotation(2), tVector3f::UnitZ()) *
					 Eigen::AngleAxisf(mesh_rotation(1), tVector3f::UnitY()) *
					 Eigen::AngleAxisf(mesh_rotation(0), tVector3f::UnitX()).toRotationMatrix();
	init_rotation_matrix.setIdentity();
	init_rotation_matrix_4x4.setIdentity();
	//init_rotation_matrix = EulerAngleRotation(init_rotation, tVector3di(2, 1, 0));
	init_rotation_matrix.setIdentity();
	init_rotation_matrix = zconventionRotation(init_rotation[2]) *
						   yconventionRotation(init_rotation[1]) *
						   xconventionRotation(init_rotation[0]);

	init_rotation_matrix_4x4.topLeftCorner<3, 3>() = init_rotation_matrix;

	neg_init_rotation_matrix.setIdentity();
	neg_init_rotation_matrix = xconventionRotation(-init_rotation[0]) *
							   yconventionRotation(-init_rotation[1]) *
							   zconventionRotation(-init_rotation[2]);
	neg_init_rotation_matrix_4x4.setIdentity();
	neg_init_rotation_matrix_4x4.topLeftCorner<3, 3>() = neg_init_rotation_matrix;

	model_matrix.setIdentity();
	mesh_matrix = model_matrix.cast<float>() * mesh_transform * scale_matrix;

	local_transform.setIdentity();
	local_transform.topLeftCorner<3, 3>() = init_rotation_matrix;
	local_transform.data()[12] = local_pos[0];
	local_transform.data()[13] = local_pos[1];
	local_transform.data()[14] = local_pos[2];
	local_transform.data()[15] = 1.0;
	// std::cout <<"[log] for link " << this->name <<" mesh transform = \n" << mesh_transform.matrix() << std::endl;
}

Mesh* BaseObject::GetMesh() const
{
	return mesh;
}

const tMatrix& BaseObject::GetModelMatrix() const
{
	return model_matrix;
}

const tVector3d& BaseObject::GetWorldPos()
{
	return pos;
}

void BaseObject::UpdateMeshMatrix()
{
	mesh_matrix = global_transform.cast<float>() * mesh_transform * scale_matrix;
}

const tMatrix4f& BaseObject::GetMeshMatrix()
{
	return mesh_matrix;
}

void BaseObject::UpdateInfo(BaseObjectJsonParam& param)
{
	name = param.name;
	local_pos = param.local_pos;
	mesh_scale = param.mesh_scale;
	mesh_rotation = param.mesh_rot;
	mesh = param.mesh;
	mass = param.mass;
	init_rotation = param.local_rot;
	shape_type = param.shape_type;
	InitObject();
	ComputeIbody();
}