#ifndef ROBOT_BASEOBJECT_H
#define ROBOT_BASEOBJECT_H

#include "ModelEigenUtils.h"
#include <string>
#include <vector>

class Mesh; // binding mesh
class RenderStruct;

enum ShapeType
{
    SPHERE_SHAPE,
    BOX_SHAPE,
    CAPSULE_SHAPE,
    CYLINDER,
    TOTAL_SHAPE_TYPE
};

const std::string shape_type_keys[TOTAL_SHAPE_TYPE] = {"sphere", "box",
                                                       "capsule", "cylinder"};

enum
{
    TRANSLATE_X = 0,
    TRANSLATE_Y,
    TRANSLATE_Z,
    REVOLUTE_X,
    REVOLUTE_Y,
    REVOLUTE_Z,
    TOTAL_ROOT_FREEDOM
};

enum
{
    REVOLUTE = 0,
    TRANSLATE
};

enum BASE_OBJ_TYPE
{
    LINK = 0,
    JOINT
};

enum JointType
{
    INVALID_JOINT = -1,
    NONE_JOINT,
    SPHERICAL_JOINT,
    REVOLUTE_JOINT,
    FIXED_JOINT,
    TOTAL_JOINT_TYPE
};
const std::string joint_type_keys[TOTAL_JOINT_TYPE] = {"none", "spherical",
                                                       "revolute", "fixed"};
struct BaseObjectParams
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    std::string name;
    Mesh *mesh;
    tVector3d local_pos;
    tVector3d local_rotation;
    tVector3f mesh_scale;
    tVector3f mesh_rotation;
    double mass;
};

struct BaseObjectJsonParam
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int id;
    int parent_id;
    std::string name;
    tVector3d local_pos;
    tVector3d local_rot;
    tVector3f mesh_scale;
    tVector3f mesh_rot;
    Mesh *mesh;
    double mass;
    tMatrix3d inertia;
    int type;
    int shape_type;
};

struct BaseObjectShapeParam
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    int shape_type;
    double top_scale;
    double bottom_scale;
    double length;
    double width;
    double height;
    BaseObjectShapeParam()
        : shape_type(0), top_scale(1.), bottom_scale(1.), length(0), width(0),
          height(0)
    {
    }
};

struct Freedom
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    double v;
    tVector3d axis;
    tVector3d grad;
    double lb;
    double ub;
    unsigned id;
    int type = REVOLUTE;
    std::string name;
    void clear_grad() { grad = tVector3d::Zero(3); }

    Freedom() = default;

    Freedom(const Freedom &other)
        : v(other.v), axis(other.axis), grad(other.grad), lb(other.lb),
          ub(other.ub), id(other.id), type(other.type), name(other.name)
    {
    }
};

class BaseObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual ~BaseObject() = default;
    BaseObject(BaseObjectParams &param);

    BaseObject(BaseObjectJsonParam &param);

    void InitObject();

    virtual void UpdateMeshMatrix();
    virtual void ComputeJacobian(const tVector3d &end_pos, tMatrix &m,
                                 int method)
    {
    }
    virtual void CleanGradient() {}
    virtual void UpdateState(bool compute_gradient = false) = 0;

    void SetRenderStruct(RenderStruct *res) { this->render_struct = res; }
    void SetLocalPos(float x, float y, float z);
    void SetLocalPos(const tVector3d &local_pos);
    const tVector3d &GetLocalPos() const { return local_pos; }
    std::string GetName() const { return name; }
    int GetNumOfChildren() const;
    void SetParent(BaseObject *parent);
    BaseObject *GetParent() const;
    BaseObject *GetFirstChild();
    tMatrix4f GetMeshMatrix();
    tMatrix3d GetMeshRotation();
    Mesh *GetMesh() const;
    tMatrix GetModelMatrix() const;
    tVector3d GetWorldPos();
    tMatrix3d GetWorldOrientation();
    virtual int GetNumOfFreedom();
    virtual Freedom *GetFreedoms(int order) { return nullptr; }
    void SetParentJoint(BaseObject *parent_joint)
    {
        this->parent_joint = parent_joint;
    }
    BaseObject *GetChild(int order);
    virtual void SetFreedomValue(int id, double v) {}
    virtual void GetFreedomValue(int id, double &v) {}
    virtual Freedom *GetFreedomByAxis(tVector3d axis, int type = REVOLUTE)
    {
        return nullptr;
    }
    virtual void SetFreedomValue(std::vector<double> &v) {}
    void SetPos(const tVector3d &pos);
    virtual void GetFreedomValue(std::vector<double> &v) {}
    void SetInertiaTensorBody(tMatrix3d &I) { this->Ibody = I; }
    const tMatrix3d &GetInertiaTensorBody() { return this->Ibody; }
    virtual bool IsJoint() const = 0;

    void AddChild(BaseObject *child);
    void ClearChildren();
    virtual void Tell();
    virtual Freedom *AddFreedom(Freedom &f) { return nullptr; }
    virtual void InitTerms() = 0;
    void InitPrevFreedomIds();
    const tVector3d &GetInitRotation() const { return init_rotation; }

    int GetNumTotalFreedoms() const { return total_freedoms; }
    virtual const tMatrixXd &GetJKDot() const { return JK_dot; }
    void SetNumTotalFreedoms(int total_freedoms)
    {
        this->total_freedoms = total_freedoms;
    }
    void GetGlobalTransform(tMatrix &m) const { m = global_transform; };
    tMatrix GetGlobalTransform() { return global_transform; };
    void GetLocalTransform(tMatrix &m) const { m = local_transform; };
    tMatrix GetLocalTransform() { return local_transform; }
    ShapeType GetShapeType() { return static_cast<ShapeType>(shape_type); }

    // ====================================================
    // modeified 04/06/20,
    const tMatrixXd &GetJKv_dq(int i) const
    {
        return jkv_dq[i];
    } // \frac{ \partial Jw_k } { \partial q_i}, dim=(3, n_freedoms)
    const tMatrixXd &GetJKw_dq(int i) const
    {
        return jkw_dq[i];
    } // \frac{ \partial Jv_k } { \partial q_i}, dim=(3, n_freedoms)
    const tMatrix &GetMWQ(int i)
    {
        return mWq[i];
    }; // \frac{\partial R} {\partial q_i} dim=(4, 4)
    virtual const tMatrix &
    GetMWQQ(int i, int j) = 0; // \frac{\partial^2 R} {\partial q_i, \partial
                               // q_i} dim=(4, 4), !! only works for joint
    //=====================================================

    virtual void ComputeJacobiByGivenPoint(tMatrixXd &j,
                                           const tVector &p) const {};
    virtual void ComputeJacobiByGivenPointTotalDOF(tMatrixXd &j,
                                                   const tVector &p) const {};
    virtual void ComputeHessianByGivenPoint(EIGEN_V_MATXD &j,
                                            const tVector &p) const {};
    virtual void ComputeLocalTransform();
    virtual void ComputeGlobalTransform();
    virtual void ComputeJKw();
    virtual void ComputeJKv();
    virtual void ComputeJK();
    virtual void ComputeJK_dot();
    virtual void ComputeJKv_dot(tVectorXd &q_dot, tVector3d &p){};
    virtual void ComputeJKw_dot(tVectorXd &q_dot){};
    virtual void ComputeDJkvdq(tVector3d &p) {}
    virtual void ComputeDJkwdq() {}

    double GetMass() const { return mass; }

    std::vector<int> &GetPrevFreedomIds() { return dependent_dof_id; };
    const tMatrix &GetNegInitRotationtMatrix() const
    {
        return neg_init_rotation_matrix_4x4;
    };

    int GetId() const { return id; }
    int GetParentId() const { return parent_id; }
    void SetLocalRot(const tVector3d &local_rot);
    void UpdateShape(BaseObjectShapeParam &param);
    void UpdateInfo(BaseObjectJsonParam &param);

    JointType GetJointType() const { return joint_type; }
    void SetJointType(JointType type) { joint_type = type; }

    void ComputeIbody();
    const tMatrixXd &GetJKw() const { return JK_w; }
    const tMatrixXd &GetJKv() const { return JK_v; }
    const tMatrixXd &GetJK() const { return JK; }
    const tMatrixXd &GetJKv_dot() const { return JK_v_dot; }
    const tMatrixXd &GetJKw_dot() const { return JK_w_dot; }
    const tMatrixXd &GetMassMatrix() const { return mass_matrix; }
    virtual void ComputeMassMatrix() {}
    void SetGlobalFreedoms(int gf) { this->global_freedom = gf; }
    int GetGlobalFreedoms() const { return global_freedom; }
    void SetOmega(tVector3d &omega) { this->omega = omega; }
    const tVector3d &GetOmega() const { return omega; }

    void SetComputeSecondDerive(bool flag)
    {
        this->compute_second_derive = flag;
    }
    bool GetComputeSecondDerive() const { return this->compute_second_derive; }

    const tVector3f &GetMeshScale() const { return mesh_scale; }

protected:
    int id;
    int parent_id;
    std::string name;                   // Name
    tVector3d local_pos;                // Local Pos wrt parent links's center
    tVector3d local_pos_parent_joint;   // Local Pos wrt parent joint's center
    tVector3d pos;                      // world pos
    BaseObject *parent;                 // parent object
    std::vector<BaseObject *> children; // children object
    tVector3f mesh_scale;               // for scaling mesh
    tVector3f mesh_rotation;            // for reset mesh rotation
    tMatrix4f mesh_matrix;              // for rendering
    tMatrix model_matrix;               // this is the rotation of the object
    tMatrix4f scale_matrix; // convert mesh_scale into transform matrix
    aff3f mesh_transform;
    tMatrix4f mesh_transform_f; // mesh initial rotation for rendering
    Mesh *mesh;                 // binding mesh
    RenderStruct *render_struct;
    double mass;
    // ============================================================================

    BaseObject *parent_joint; // For joint, this variable is the parent of the
                              // parent link, for link it is jut the parent
    tMatrix3d orientation;    // 3x3 matrix of local orientation: rz * ry * rx
    EIGEN_V_tMatrixD mTq;     // \partial local  transform over \partial q
    EIGEN_V_tMatrixD mWq;     // \partial global transform over \partial q
    int total_freedoms;       // total_freedom = prev_freedom + local_freedom
    int prev_freedoms;
    std::vector<int> dependent_dof_id;
    int local_freedom;
    tMatrix local_transform;
    tMatrix global_transform;

    // ============================================================================
    // fixed skeleton rotation vars
    tVector3d init_rotation;
    tMatrix3d init_rotation_matrix; // rest pose
    tMatrix init_rotation_matrix_4x4;
    tMatrix3d neg_init_rotation_matrix; // rest pose
    tMatrix neg_init_rotation_matrix_4x4;
    tMatrix rot_parent_child; // child to parent

    // ============================================================================

    JointType joint_type;

    // ========================For Torque
    // minimization=============================
    tMatrix3d Ibody;
    int shape_type;
    tMatrixXd JK_w;
    tMatrixXd JK_v;
    tMatrixXd JK_v_dot;
    tMatrixXd JK_w_dot;
    tMatrixXd JK;
    tMatrixXd JK_dot;
    tMatrixXd mass_matrix;
    int global_freedom; // global freedom is all freedoms of the whole character
    tVector3d omega;

    EIGEN_V_MATXD jkv_dq; // 3 nxn
    EIGEN_V_MATXD jkw_dq;

    // ============================================================================

    bool compute_second_derive;
};

#endif // ROBOT_BASEOBJECT_H
