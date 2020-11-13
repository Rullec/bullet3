#include "BaseObject.h"
#include "EulerAngelRotationMatrix.h"
#include "tools.h"
#include <iostream>

BaseObjectParams::BaseObjectParams()
{
    name = "uninitedc_base_object_name";
    mesh = nullptr;
    local_pos.setZero();
    local_rotation.setZero();
    mesh_scale.setZero();
    mesh_rotation.setZero();
    mass = 0;
}
BaseObjectJsonParam::BaseObjectJsonParam()
{
    id = -1;
    parent_id = -1;
    name = " uninited_base_object_json_name ";
    local_pos.setZero();
    local_rot.setZero();
    mesh_scale.setZero();
    mesh_rot.setZero();
    mesh = nullptr;
    mass = 0;
    inertia.setZero();
    type = -1;
    shape_type = -1;
}

/**
 * \brief           Get nxn matrix of dJkv/dqi
 * the shape of the total tensor dJkvdq is 3xnxn. 
 * Now we storage it in an 3-elements array dJkvdq, and each of it is a nxn matrix
 * Note that n is "total_freedoms" but not "global_freedoms"
 * This API should be used with care, it is not very intuitive
*/
const tMatrixXd &BaseObject::GetdJKvdq_nxnversion(int i) const
{
    return jkv_dq[i];
}

/**
 * \brief           Get nxn matrix of dJkw/dqi 
 * the shape of the total tensor dJkwdq is 3xnxn. 
 * Now we storage it in an 3-elements array dJkwdq, and each of it is a nxn matrix
 * Note that n is "total_freedoms" but not "global_freedoms"
 * This API should be used with care, it is not very intuitive
*/
const tMatrixXd &BaseObject::GetdJKwdq_nxnversion(int i) const
{
    return jkw_dq[i];
}

/**
 * \brief           Get the 6xn Jk for this object (usualy links)
 * Jk = [Jkv \\ Jkw] \in R^{6 \times n}
 * n is the total_freedoms but not the global freedoms
 * \param dof       dJk/dqdot, in total_freedoms
*/
tMatrixXd BaseObject::GetdJkdq_6xnversion(int dof) const
{
    assert(dof < total_freedoms);
    tMatrixXd dJkdqi = tMatrixXd::Zero(6, total_freedoms);
    dJkdqi.block(0, 0, 3, total_freedoms) = GetdJKvdq_3xnversion(dof);
    dJkdqi.block(3, 0, 3, total_freedoms) = GetdJKwdq_3xnversion(dof);
    return dJkdqi;
}

/**
 * \brief           Get the 3xn Jkv for this object (usually links)
 * \param dof       interested freedom in total_freedom
 * In default case, this Jkv is definied at the COM of this link, but not other places
 * Jkv = d(position)/dq \in 3 \times n
 * n is the total freedom but not the global_freedom
*/
tMatrixXd BaseObject::GetdJKvdq_3xnversion(int dof) const
{
    assert(dof < total_freedoms);
    tMatrixXd dJkvdq = tMatrixXd::Zero(3, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        dJkvdq(0, i) = jkv_dq[0](dependent_dof_id[i], dependent_dof_id[dof]);
        dJkvdq(1, i) = jkv_dq[1](dependent_dof_id[i], dependent_dof_id[dof]);
        dJkvdq(2, i) = jkv_dq[2](dependent_dof_id[i], dependent_dof_id[dof]);
    }
    return dJkvdq;
}

/**
 * \brief           Get the 3xn Jkw for this object (usually links)
 * \param dof       total_freedoms
 * Jkw = d([\dot{R}*RT]^{-1})/dq \in 3 \times n
 * n is the total freedom but not the global_freedom
 * []^{-1} means the extraction of skew vector
*/
tMatrixXd BaseObject::GetdJKwdq_3xnversion(int dof) const
{
    assert(dof < total_freedoms);
    tMatrixXd dJkwdq = tMatrixXd::Zero(3, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        dJkwdq(0, i) = jkw_dq[0](dependent_dof_id[i], dependent_dof_id[dof]);
        dJkwdq(1, i) = jkw_dq[1](dependent_dof_id[i], dependent_dof_id[dof]);
        dJkwdq(2, i) = jkw_dq[2](dependent_dof_id[i], dependent_dof_id[dof]);
    }
    return dJkwdq;
}
/**
 * \brief       Get \frac{\partial R} {\partial q_i} dim=(4, 4)
 * \param i     the local freedom id in this base object, not the global freedom
 * id
 */
const tMatrix &BaseObject::GetMWQ(int i) const
{
    if (i >= total_freedoms)
    {
        std::cout << "[error] BaseObject GetMWQ for dof " << i
                  << " but the total freedoms is " << total_freedoms
                  << std::endl;
        exit(0);
    }
    return mWq[i];
}

int BaseObject::GetNumOfChildren() const
{
    return static_cast<int>(children.size());
}

void BaseObject::SetParent(BaseObject *parent) { BaseObject::parent = parent; }

void BaseObject::SetLocalPos(float x, float y, float z)
{
    local_pos[0] = x;
    local_pos[1] = y;
    local_pos[2] = z;
}

void BaseObject::SetLocalPos(const tVector3d &local_pos)
{
    this->local_pos = local_pos;
}

int BaseObject::GetNumOfFreedom() { return -1; }

BaseObject *BaseObject::GetChild(int order)
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

void BaseObject::SetPos(const tVector3d &pos) { this->pos = pos; }

void BaseObject::AddChild(BaseObject *ch) { children.push_back(ch); }

void BaseObject::ClearChildren() { children.clear(); }

BaseObject *BaseObject::GetParent() const { return parent; }

void BaseObject::Tell()
{
    std::cout << name << ":\nlocal_pos: " << local_pos[0] << ", "
              << local_pos[1] << ", " << local_pos[2] << "\n";
}

/**
 * \brief           Add parent's dof into the "dependent_dof_id" array. 
 * Note that the "dependent_dof_id" is still incomplete and needs to add my own freedoms after this function
*/
void BaseObject::InitPrevFreedomIds()
{
    auto &parent_prev_ids = parent->GetPrevFreedomIds();
    for (size_t i = 0; i < parent_prev_ids.size(); i++)
    {
        dependent_dof_id.push_back(parent_prev_ids[i]);
    }
}

/**
 * \brief           Build the map from global freedom id to total freedom id which this object is dependent of 
 * 
 * The time of calling this method is critical: the dependent_dof_id must be initialized completely.
*/
void BaseObject::InitGlobalToTotalFreedomMap()
{
    this->map_from_global_to_total_freedom.clear();
    for (int i = 0; i < dependent_dof_id.size(); i++)
    {
        int global_dof_id = dependent_dof_id[i];
        int total_dof_id = i;
        map_from_global_to_total_freedom[global_dof_id] = total_dof_id;
    }
}

void BaseObject::SetLocalRot(const tVector3d &local_rot)
{
    init_rotation = local_rot;
    init_rotation_matrix.setIdentity();
    init_rotation_matrix_4x4.setIdentity();
    init_rotation_matrix = zconventionRotation(init_rotation[2]) *
                           yconventionRotation(init_rotation[1]) *
                           xconventionRotation(init_rotation[0]);

    init_rotation_matrix_4x4.topLeftCorner<3, 3>() = init_rotation_matrix;
}

void BaseObject::UpdateShape(BaseObjectShapeParam &param)
{
    shape_type = param.shape_type;
    if (param.width != 0)
        mesh_scale(0) = param.width;
    if (param.length != 0)
        mesh_scale(2) = param.length;
    if (param.height != 0)
        mesh_scale(1) = param.height;
    scale_matrix = Eigen::scale(mesh_scale.x(), mesh_scale.y(), mesh_scale.z());

    // only compute inertia for links
    if (!IsJoint())
        ComputeIbody();
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
        double radius = mesh_scale(0) / 2;
        Ibody = tMatrix3d::Identity() * 2.0 / 5 * mass * pow(radius, 2);
    }
    else if (shape_type == ShapeType::CAPSULE_SHAPE)
    {
        Ibody = tMatrix3d::Zero();
        double radius = mesh_scale(0) / 2;
        double height = mesh_scale(1);

        tVector3d halfExtents(radius, radius, radius);
        halfExtents[1] += height / 2;

        double lx = 2 * (halfExtents[0]);
        double ly = 2 * (halfExtents[1]);
        double lz = 2 * (halfExtents[2]);
        const double x2 = lx * lx;
        const double y2 = ly * ly;
        const double z2 = lz * lz;
        const double scaledmass = mass / 12;

        Ibody(0, 0) = scaledmass * (y2 + z2);
        Ibody(1, 1) = scaledmass * (x2 + z2);
        Ibody(2, 2) = scaledmass * (x2 + y2);

        // std::cout << "[log] capsule radius " << radius << " height " << height
        //           << " inertia = \n"
        //           << Ibody << std::endl;
    }
    else
    {
        std::cout << "Unsupported type: " << shape_type << " in ComputeIbody\n";
        exit(0);
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
        const tMatrix &m = parent->GetGlobalTransform();
        // global_transform = m * local_transform;
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
    // std::cout << "-----begin compute jkw-----\n";
    JK_w.setZero();
    // std::cout << "global trans = \n"
    // 		  << global_transform << std::endl;
    for (int i = 0; i < total_freedoms; ++i)
    {
        tMatrix3d w_skesym = mWq[i].topLeftCorner<3, 3>() *
                             global_transform.topLeftCorner<3, 3>().transpose();
        JK_w.col(dependent_dof_id[i]) = Tools::FromSkewSymmetric(w_skesym);
        // std::cout << "dof " << i << " mWq \n" << mWq[i] << std::endl;
        // std::cout << "dof " << i << " global trans = \n"
        // 		  << global_transform << std::endl;
        // std::cout << "dof " << i << " w = \n"
        // 		  << w_skesym << std::endl;
        // std::cout << "dof " << i << " Jkw = \n"
        // 		  << JK_w.col(dependent_dof_id[i]).transpose() <<
        // std::endl;
    }
    // std::cout << "Jkw = \n"
    // 		  << JK_w << std::endl;
    // std::cout << "-----end compute jkw-----\n";
}

void BaseObject::ComputeJKv()
{
    JK_v.setZero();
    tVector p = tVector(0, 0, 0, 1);
    tVector r;
    for (int i = 0; i < total_freedoms; ++i)
    {
        Tools::MatMul4x1(mWq[i], p, r);
        int dependent_id = dependent_dof_id[i];
        JK_v.col(dependent_id) = Tools::GettVector3d(r);
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

BaseObject *BaseObject::GetFirstChild()
{
    if (children.empty())
        return nullptr;
    return children[0];
}

BaseObject::BaseObject(const BaseObjectParams &param)
    : mId(0), parent_id(-1), name(param.name), local_pos(param.local_pos),
      parent(nullptr), mesh_scale(param.mesh_scale),
      mesh_rotation(param.mesh_rotation), mesh(param.mesh),
      render_struct(nullptr), mass(param.mass), parent_joint(nullptr),
      total_freedoms(-1), prev_freedoms(0), local_freedom(0),
      init_rotation(param.local_rotation), joint_type(JointType::INVALID_JOINT),
      shape_type(-1), global_freedom(0), compute_second_derive(false),
      compute_third_derive(false)
{
    scale_matrix = Eigen::scale(mesh_scale.x(), mesh_scale.y(), mesh_scale.z());

    mesh_transform = Eigen::AngleAxisf(mesh_rotation(2), tVector3f::UnitZ()) *
                     Eigen::AngleAxisf(mesh_rotation(1), tVector3f::UnitY()) *
                     Eigen::AngleAxisf(mesh_rotation(0), tVector3f::UnitX())
                         .toRotationMatrix();

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
    neg_init_rotation_matrix_4x4.topLeftCorner<3, 3>() =
        neg_init_rotation_matrix;

    model_matrix = tMatrix::Identity();
    mesh_matrix = model_matrix.cast<float>() * mesh_transform * scale_matrix;

    local_transform.setIdentity();
    local_transform.topLeftCorner<3, 3>() = init_rotation_matrix;
    local_transform.data()[12] = local_pos[0];
    local_transform.data()[13] = local_pos[1];
    local_transform.data()[14] = local_pos[2];
    local_transform.data()[15] = 1.0;
}

BaseObject::BaseObject(const BaseObjectJsonParam &param)
    : mId(param.id), parent_id(param.parent_id), name(param.name),
      local_pos(param.local_pos), parent(nullptr), mesh_scale(param.mesh_scale),
      mesh_rotation(param.mesh_rot), mesh(param.mesh), render_struct(nullptr),
      mass(param.mass), parent_joint(nullptr), total_freedoms(-1),
      prev_freedoms(0), local_freedom(0), init_rotation(param.local_rot),
      joint_type(JointType::INVALID_JOINT), Ibody(param.inertia),
      shape_type(param.shape_type), global_freedom(0),
      compute_second_derive(false), compute_third_derive(false)
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
                     Eigen::AngleAxisf(mesh_rotation(0), tVector3f::UnitX())
                         .toRotationMatrix();
    init_rotation_matrix.setIdentity();
    init_rotation_matrix_4x4.setIdentity();
    // init_rotation_matrix = EulerAngleRotation(init_rotation, tVector3di(2, 1,
    // 0));
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
    neg_init_rotation_matrix_4x4.topLeftCorner<3, 3>() =
        neg_init_rotation_matrix;

    model_matrix.setIdentity();
    mesh_matrix = model_matrix.cast<float>() * mesh_transform * scale_matrix;

    local_transform.setIdentity();
    local_transform.topLeftCorner<3, 3>() = init_rotation_matrix;
    local_transform.data()[12] = local_pos[0];
    local_transform.data()[13] = local_pos[1];
    local_transform.data()[14] = local_pos[2];
    local_transform.data()[15] = 1.0;
    // std::cout <<"[log] for link " << this->name <<" mesh transform = \n" <<
    // mesh_transform.matrix() << std::endl;
}

Mesh *BaseObject::GetMesh() const { return mesh; }

tMatrix BaseObject::GetModelMatrix() const { return model_matrix; }

tVector3d BaseObject::GetWorldPos() { return pos; }

tMatrix3d BaseObject::GetWorldOrientation()
{
    return global_transform.topLeftCorner<3, 3>();
}

void BaseObject::UpdateMeshMatrix()
{
    mesh_matrix =
        global_transform.cast<float>() * mesh_transform * scale_matrix;
}

tMatrix4f BaseObject::GetMeshMatrix() { return mesh_matrix; }
tMatrix3d BaseObject::GetMeshRotation()
{
    return mesh_transform.matrix().cast<double>().block(0, 0, 3, 3);
}

void BaseObject::UpdateInfo(BaseObjectJsonParam &param)
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

void BaseObject::SetComputeThirdDerive(bool flag)
{
    compute_third_derive = flag;
}
bool BaseObject::GetComputeThirdDeriv() const { return compute_third_derive; }

void BaseObject::SetComputeSecondDerive(bool flag)
{
    compute_second_derive = flag;
}
bool BaseObject::GetComputeSecondDerive() const
{
    return this->compute_second_derive;
}

const tMatrixXd &BaseObject::GetJKvdot() const { return JK_v_dot; }
const tMatrixXd &BaseObject::GetJKwdot() const { return JK_w_dot; }

tMatrixXd BaseObject::GetJKvdot_reduced() const
{
    tMatrixXd Jkv_dot_reduced = tMatrixXd::Zero(3, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
        Jkv_dot_reduced.col(i) = JK_v_dot.col(dependent_dof_id[i]);
    return Jkv_dot_reduced;
}
tMatrixXd BaseObject::GetJKwdot_reduced() const
{
    tMatrixXd Jkw_dot_reduced = tMatrixXd::Zero(3, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
        Jkw_dot_reduced.col(i) = JK_w_dot.col(dependent_dof_id[i]);
    return Jkw_dot_reduced;
}

tMatrixXd BaseObject::GetJKw_reduced() const
{
    tMatrixXd Jkw_reduced = tMatrixXd::Zero(3, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        Jkw_reduced.col(i) = JK_w.col(dependent_dof_id[i]);
    }
    return Jkw_reduced;
}
tMatrixXd BaseObject::GetJKv_reduced() const
{
    tMatrixXd Jkv_reduced = tMatrixXd::Zero(3, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        Jkv_reduced.col(i) = JK_v.col(dependent_dof_id[i]);
    }
    return Jkv_reduced;
}
tMatrixXd BaseObject::GetJK_reduced() const
{
    tMatrixXd Jk_reduced = tMatrixXd::Zero(6, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        Jk_reduced.col(i) = this->JK.col(dependent_dof_id[i]);
    }
    return Jk_reduced;
}

tMatrixXd BaseObject::GetJkdot_recuded() const
{
    tMatrixXd Jk_dot_reduced = tMatrixXd::Zero(6, total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        Jk_dot_reduced.col(i) = JK_dot.col(dependent_dof_id[i]);
    }
    return Jk_dot_reduced;
}
tVectorXd BaseObject::GetShortedFreedom(const tVectorXd &qx_log) const
{
    assert(qx_log.size() == global_freedom);
    tVectorXd q = tVectorXd::Zero(total_freedoms);
    for (int i = 0; i < total_freedoms; i++)
    {
        q[i] = qx_log[dependent_dof_id[i]];
    }
    return q;
}