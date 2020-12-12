#ifndef ROBOT_BASEOBJECT_H
#define ROBOT_BASEOBJECT_H

#include "ModelEigenUtils.h"
#include <map>
#include <string>
#include <vector>

class Mesh; // binding mesh
class RenderStruct;

enum ShapeType
{
    INVALID_SHAPE_TYPE = -1,
    SPHERE_SHAPE = 0,
    BOX_SHAPE,
    CAPSULE_SHAPE,
    CYLINDER,
    TOTAL_SHAPE_TYPE
};

const std::string shape_type_keys[TOTAL_SHAPE_TYPE] = {"sphere", "box",
                                                       "capsule", "cylinder"};
ShapeType BuildShapeType(std::string string);

enum eRootFreedomEnum
{
    TRANSLATE_X = 0,
    TRANSLATE_Y,
    TRANSLATE_Z,
    REVOLUTE_X,
    REVOLUTE_Y,
    REVOLUTE_Z,
    TOTAL_ROOT_FREEDOM
};

enum eFreedomType
{
    REVOLUTE = 0,    // euler angle revolute freedom
    TRANSLATE,       // translation freedom
    EXPONENTIAL_MAP, // exponential map freedom
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
    LIMIT_NONE_JOINT,
    BIPEDAL_NONE_JOINT,
    FIXED_NONE_JOINT,
    SPHERICAL_JOINT,
    REVOLUTE_JOINT,
    UNIVERSAL_JOINT,
    BALLINSOCKET_JOINT,
    FIXED_JOINT,
    TOTAL_JOINT_TYPE
};
const std::string joint_type_keys[TOTAL_JOINT_TYPE] = {
    "none",     "limit_none", "bipedal_none",   "fixed_none", "spherical",
    "revolute", "universal",  "ball_in_socket", "fixed"};
JointType BuildJointType(std::string name);

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
    BaseObjectParams();
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
    BaseObjectJsonParam();
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
    double vdot;
    tVector3d axis;
    tVector3d grad;
    double lb;
    double ub;
    unsigned id; // global freedom id in the whole multibody
    int type = REVOLUTE;
    std::string name;
    void clear_grad() { grad = tVector3d::Zero(3); }

    Freedom() = default;

    Freedom(const Freedom &other)
        : v(other.v), vdot(other.vdot), axis(other.axis), grad(other.grad),
          lb(other.lb), ub(other.ub), id(other.id), type(other.type),
          name(other.name)
    {
    }
};

class BaseObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual ~BaseObject() = default;
    BaseObject(const BaseObjectParams &param);

    BaseObject(const BaseObjectJsonParam &param);

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
    virtual int GetNumOfFreedom() const;
    virtual Freedom *GetFreedoms(int order) { return nullptr; }
    void SetParentJoint(BaseObject *parent_joint)
    {
        this->parent_joint = parent_joint;
    }
    BaseObject *GetChild(int order);
    virtual void SetFreedomValue(int id, double v) {}
    virtual void GetFreedomValue(int id, double &v) const {}
    virtual void SetFreedomValueDot(int id, double v) = 0;
    virtual void GetFreedomValueDot(int id, double &v) const = 0;
    virtual Freedom *GetFreedomByAxis(tVector3d axis, int type = REVOLUTE)
    {
        return nullptr;
    }
    virtual void SetFreedomValue(std::vector<double> &v) {}
    virtual void GetFreedomValue(std::vector<double> &v) const {}
    virtual void SetFreedomValueDot(std::vector<double> &v) = 0;
    virtual void GetFreedomValueDot(std::vector<double> &v) const = 0;
    void SetPos(const tVector3d &pos);
    void SetInertiaTensorBody(tMatrix3d &I) { this->Ibody = I; }
    const tMatrix3d &GetInertiaTensorBody() { return this->Ibody; }
    virtual bool IsJoint() const = 0;

    void AddChild(BaseObject *child);
    void ClearChildren();
    virtual void Tell();
    virtual Freedom *AddFreedom(Freedom &f) { return nullptr; }
    virtual void InitTerms() = 0;
    void InitPrevFreedomIds();
    void InitGlobalToTotalFreedomMap();
    const tVector3d &GetInitRotation() const { return init_rotation; }

    int GetNumTotalFreedoms() const { return total_freedoms; }
    void SetNumTotalFreedoms(int total_freedoms)
    {
        this->total_freedoms = total_freedoms;
    }
    void GetGlobalTransform(tMatrix &m) const { m = global_transform; };
    tMatrix GetGlobalTransform() { return global_transform; };
    void GetLocalTransform(tMatrix &m) const { m = local_transform; };
    tMatrix GetLocalTransform() { return local_transform; }
    ShapeType GetShapeType() { return static_cast<ShapeType>(shape_type); }
    virtual bool GetIsRootJoint() const = 0;
    // ====================================================
    // modeified 04/06/20,
    const tMatrixXd &GetdJKvdq_nxnversion(int i) const;
    const tMatrixXd &GetdJKwdq_nxnversion(int i) const;
    tMatrixXd GetdJkdq_6xnversion(int i)
        const; // needs to crate new matrix, so we cannot return a const reference
    tMatrixXd GetdJKvdq_3xnversion(int i) const;
    tMatrixXd GetdJKwdq_3xnversion(int i) const;
    const tMatrix &GetMWQ(int i) const;
    virtual const tMatrix &GetMWQQ(int i, int j) const = 0;
    virtual const tMatrix &
    GetMWQQQ(int i, int j,
             int k) const = 0; // please check the comment of variable mWqqq
    //=====================================================

    virtual void ComputeJacobiByGivenPoint(tMatrixXd &j,
                                           const tVector &p) const {};
    virtual void ComputeJacobiByGivenPointTotalDOF(tMatrixXd &j,
                                                   const tVector &p) const {};
    virtual void ComputeHessianByGivenPoint(EIGEN_V_MATXD &j,
                                            const tVector &p) const {};
    virtual void ComputeLocalTransform();
    virtual void ComputeGlobalTransform();
    virtual void
    ComputeLocalJkw() = 0;     // compute local jkw, w.r.t only himself's dof
    virtual void ComputeJKw(); // compute global jkw, w.r.t all dependent dof
    virtual void ComputeJKv();
    virtual void ComputeJK();
    virtual void ComputeJK_dot();
    virtual void ComputeJKv_dot(const tVectorXd &q_dot, const tVector3d &p){};
    virtual void ComputeJKw_dot(const tVectorXd &q_dot){};
    virtual void ComputedJkvdot_dq(const tVectorXd &qdot, const tVector3d &p){};
    virtual void ComputedJkwdot_dq(const tVectorXd &qdot){};
    virtual void ComputeDJkvdq(const tVector3d &p) {}
    virtual void ComputeDJkwdq() {}
    virtual void ComputeDDJkvddq(const tVector3d &p){};
    virtual void ComputeDDJkwdqq(){};
    double GetMass() const { return mass; }

    std::vector<int> &GetPrevFreedomIds() { return dependent_dof_id; };
    const tMatrix &GetNegInitRotationtMatrix() const
    {
        return neg_init_rotation_matrix_4x4;
    };

    int GetId() const { return mId; }
    int GetParentId() const { return parent_id; }
    void SetLocalRot(const tVector3d &local_rot);
    void UpdateShape(BaseObjectShapeParam &param);
    void UpdateInfo(BaseObjectJsonParam &param);

    JointType GetJointType() const { return joint_type; }
    void SetJointType(JointType type) { joint_type = type; }

    void ComputeIbody();
    virtual const tMatrixXd &GetLocalJkw() const = 0;
    const tMatrixXd &GetJKw() const { return JK_w; }
    const tMatrixXd &GetJKv() const { return JK_v; }
    const tMatrixXd &GetJK() const { return JK; }
    const tMatrixXd &GetJKvdot() const;
    const tMatrixXd &GetJKwdot() const;
    virtual const tMatrixXd &GetJKDot() const { return JK_dot; }
    tMatrixXd GetJKw_reduced() const;
    tMatrixXd GetJKv_reduced() const;
    tMatrixXd GetJK_reduced() const;
    tMatrixXd GetJKvdot_reduced() const;
    tMatrixXd GetJKwdot_reduced() const;
    tMatrixXd GetJkdot_recuded() const;
    const tMatrixXd &GetMassMatrix() const { return mass_matrix; }
    virtual void ComputeMassMatrix() {}
    void SetGlobalFreedoms(int gf) { this->global_freedom = gf; }
    int GetGlobalFreedoms() const { return global_freedom; }
    void SetOmega(tVector3d &omega) { this->omega = omega; }
    const tVector3d &GetOmega() const { return omega; }

    void SetComputeSecondDerive(bool flag);
    bool GetComputeSecondDerive() const;
    void SetComputeThirdDerive(bool flag);
    bool GetComputeThirdDeriv() const;
    const tVector3f &GetMeshScale() const { return mesh_scale; }
    tVectorXd GetShortedFreedom(const tVectorXd &qx_log) const;

protected:
    int mId;
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
    int prev_freedoms;        // the freedoms owned by my parent joints
    std::vector<int>
        dependent_dof_id; // map from total_freedom_id to global_freedom_id
    std::map<int, int>
        map_from_global_to_total_freedom; // map from global_freedom_id to total_freedom_id
    int local_freedom;                    // the freedom owned by myself
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

    // ========================For Torque minimization=============================
    tMatrix3d Ibody;
    int shape_type;
    tMatrixXd
        JK_w_local; // local jkw in joint's frame, w.r.t joint himself's freedom
    tMatrixXd JK_w; // global jkw in world frame, w.r.t all dependent freedoms
    tMatrixXd JK_v;
    tMatrixXd JK_v_dot; // \dot{Jkv} in global freedoms
    tMatrixXd JK_w_dot;
    tMatrixXd JK;
    tMatrixXd JK_dot;
    tMatrixXd mass_matrix;
    int global_freedom; // global freedom is all freedoms of the whole character
    tVector3d omega;

    EIGEN_V_MATXD
    jkv_dq; // vector<tMatrix: nxn>, vector size = 3 (one matrix per channel), d(Jv)/dq, global_freedoms
    EIGEN_V_MATXD
    jkw_dq; // vector<tMatrix: nxn>, vector size = 3(one matrix per channel), d(Jw)/dq, global_freedoms
    EIGEN_VV_MATXD
    ddjkv_dqq; // vector<vector<tMatrix: 3xn>>, the index of two outer layer is the freedom index, d^2(Jv)/d(qiqj), only store the lower diagnoal, total_freedoms
    EIGEN_VV_MATXD
    ddjkw_dqq; // vector<vector<tMatrix: 3xn>>, the index of two outer layer is the freedom index, d^2(Jw)/d(qiqj), only store the lower diagnoal, total_freedoms

    EIGEN_V_MATXD
    dJkvdot_dq; // vector<tMatrixXd : 3xn>, where n is the total_freedoms but not global_freedoms. d(Jkvdot)/dq, total_freedoms
    EIGEN_V_MATXD
    dJkwdot_dq; // vector<tMatrixXd : 3xn>, where n is the total_freedoms but not global_freedoms. d(JKwdot)/dq, total_freedoms

    // ============================================================================

    bool
        compute_second_derive; // switch for calculating second-order derivatives
    bool compute_third_derive; // switch for calculating third-order derivatives
};

#endif // ROBOT_BASEOBJECT_H
