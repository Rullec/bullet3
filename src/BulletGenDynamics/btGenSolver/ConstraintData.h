#pragma once
#include "../btGenModel/ColObjBase.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class btGenRigidBody;
struct btGenRobotCollider;
class cRobotModelDynamics;
#include <map>
enum btGenLCPConstraintType
{
    BTGEN_LCP_CONTACT_PAIR_CONS = 0,
    BTGEN_LCP_JOINT_LIMIT_CONS
};
struct btGenLCPConstraintData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    tMatrixXd jac0, jac1;
    int constraint_id; // constraint id, for btGenContactPairData it's the contact_pair_id
    const btGenLCPConstraintType mConstraintType;
    virtual void CalcJacobian() = 0;

protected:
    btGenLCPConstraintData(int constraint_id, btGenLCPConstraintType type);
};

struct btGenJointLimitData : public btGenLCPConstraintData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenJointLimitData(int constraint_id, cRobotModelDynamics *multibody,
                        int dof_id, bool is_upper_bound, double violate_value);

    void ApplyGeneralizedForce(double val);
    void ApplyJointTorque(double val);
    virtual void CalcJacobian() override;
    double violate_value;
    cRobotModelDynamics *multibody;
    double dt;
    int dof_id;
    bool is_upper_bound;
    tVector3d joint_direction;
};

class btPersistentManifold;

/**
 * \brief       contact pair data structure
 *      A contact pair can represent a contact point 
 *      which has two contact positions on two different collision objects, respectively
*/
struct btGenContactPairData : public btGenLCPConstraintData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenContactPairData(int c_id);
    void Init(double dt, btPersistentManifold *manifold,
              int contact_id_in_manifold);
    virtual void CalcJacobian() override;
    double dt; // timestep
    btGenCollisionObject *mBodyA,
        *mBodyB;                // the bodyA and bodyB in this contact pair
    eColObjType mTypeA, mTypeB; // the type of bodyA&B
    tVector
        mNormalPointToA; // unit normal vector from bodyB to bodyA at this contact pair
    tVector mContactPosOnA, mContactPosOnB;
    double distance;
    bool mIsMBSelfCollision;

    int mBodyId0, mBodyId1;
    int mbody0GroupId, mbody1GroupId;
    tMatrixXd mD; // friction cone matrix 3xN, N is num of frictins
    tMatrixXd mS; // convert matrix from x0 \in R^{N+2} to f0 \in R^{3}, S.shape
                  // = (3, N+2)
    int mNumOfFriction; // friction number

    double mSI_normal_contact_value;     // total normal contact magnitute
    tVector mSI_tangent_contact_dir[2];  // 2 tangent friction directions
    double mSI_tangent_contact_value[2]; // tangent friction magnitutes on 2
                                         // directions

    // u_rel = Z * f + h
    tMatrix mSI_Z; // mat H convert the cartesian force to relative velocity
    tVector mSI_h; // vec h convert force to relative vel

    void TestHt(int n);
    void Setup(int num_frictions);
    void ApplyForceResultVector(const tVectorXd &x0);
    void ApplyForceCartersian(const tVector &force);

    bool CheckOverlap(btGenContactPairData *other_data);
    tMatrixXd GetJacobian(int world_col_id);
    tMatrixXd GetConvertMatS(int world_col_id);
    int GetBody0Id();
    int GetBody1Id();
    tVector GetRelVel();
    tVector GetVelOnBody0();
    tVector GetVelOnBody1();
    void UpdateVel(double dt);

protected:
    void CalcBodyInfo();

    void CalcFrictionCone(tMatrixXd &D);
    void CalcConvertMat(tMatrixXd &S);
};

/**
 * \brief           Collision Group data structure
 *     
 * 1. What is a collision group:
 *      Briefly, A collision group is a data structure for a/multiple collision objects
 *      1. if this collision group work for a multibody, then it will include all RobotLinkColliders of this multibody
 *      2. if this collision group work for a rigidbody, it will only include a single rigidbody itself
 * 
 * 2. Why we need collision group?
 *      In order to make the calculation of LCP matrix A&b efficiently. 
 *      Because we need to calculate & reuse the jacobian of each contact point of a multibody
 * 
 * 3. the construction of CollisionGroup
 *      for multibody case: it will be constructed by ANY of a robot collider 
 *      for rigidbody case: it will be constructed by this rigidbody
 * 
 * 4. What is "ConvertCartesianForceToVelocityMat" and "ConvertCartesianForceToVelocityVec"
 *      this pair of mat & vec can convert 
 *          "the solution of LCP cartesian contact force"
 *          to 
 *          "the abs vel at each nonself-collision contact pair" & "the rel vel at each self-collision contact pair"
 *      it is a complex, dense mat made up by jacobians
*/
struct btGenCollisionGroupData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenCollisionGroupData(int gid, btGenCollisionObject *,
                            bool record_cartesian_convert_mat_of_residual);
    void AddContactPair(btGenContactPairData *, bool isbody0);
    void AddJointLimitConstraint(btGenJointLimitData *);
    void Clear();
    void Setup(int n_total_contacts, int n_total_jointlimits);
    void
    GetCharacterConvertMatOfResidualRecord(tEigenArr<tMatrixXd> &record) const;
    void PrintDebugInfo() const;
    btGenCollisionObject
        *mBody; // the representing collision object of this collision group
    tMatrixXd mConvertCartesianForceToVelocityMat;
    tVectorXd mConvertCartesianForceToVelocityVec;

    tMatrixXd mSI_ConvertCartesianForceToVelocityMat;
    tVectorXd mSI_ConvertCartesianForceToVelocityVec;

protected:
    int mGroupId;
    std::vector<btGenJointLimitData *> mJointLimits; // joint limit constraint
    std::vector<btGenContactPairData *>
        mContactPairs; // contact pair constraint
    std::vector<bool> mIsBody0;
    int num_world_contact_pairs; // the number of contact pairs in this world
    int num_world_joint_limits;  // the number of joint limits in this world
    int num_world_constraints;   // the sum of two items above
    void SetupRobotCollider();
    void SetupRigidBody();
    void SetupRobotColliderVars();
    void GetContactJacobianArrayRobotCollider(
        tEigenArr<tMatrixXd> &nonself_contact_jac_list,
        tEigenArr<tMatrixXd> &selfcontact_jac_list);
    void
    GetJointLimitJaocibanArrayRobotCollider(tEigenArr<tMatrixXd> &jac_list);
    tMatrixXd CalcRobotColliderMiddlePart(double dt) const;
    tVectorXd CalcRobotColliderResidual(double dt) const;
    tVectorXd CalcRobotColliderResidualOldSemiImplicit(double dt) const;
    tVectorXd CalcRobotColliderResidualNewSemiImplicit(double dt) const;
    tMatrixXd CalcRobotColliderJacPartBPrefix(double dt) const;
    void GetDataInfoRobotCollider(int constraint_id_ingroup,
                                  int &constraint_st_pos_world,
                                  int &constraint_size_world) const;
    tMatrixXd
    GetAbsVelConvertMat_JacPartA_RobotCollider(int cons_id_ingroup) const;
    tMatrixXd
    GetAbsVelConvertMat_JacPartB_RobotCollider(int cons_id_ingroup) const;
    // buffer for multibody
    tMatrixXd M, inv_M, coriolis_mat, damping_mat, M_dt_C_D_inv;

    // vars for multibody
    int num_contact_pairs_ingroup; // the number of contact pairs involved in this group
    int num_self_contact_pairs_ingroup; // the number of self contact pairs inside of this group
    int num_nonself_contact_pairs_ingroup; // the number of non-self contact pairs involved in this group
    int num_joint_constraint_ingroup; // the number of group joint constraints

    // /*
    //     map the world contact id to the index in "mContactPairs" aka (local)_contact_id or contact_id_ingroup
    // */
    // std::map<int, int> map_world_contact_id_to_local_contact_id;

    // /*
    //     map the local_contact_id_ingroup to the index of local_self_contact_id
    // */
    // std::map<int, int> map_local_contact_id_to_self_contact_id;

    // /*
    //     map the local_contact_id_ingroup to the index of local_nonself_contact_id
    // */
    // std::map<int, int> map_local_contact_id_to_nonself_contact_id;

    // record the convert mat from gen vel to cartesian vel for diffMBRL
    bool mEnableRecordCartesianConvertMatOfResidual;
    tEigenArr<tMatrixXd> mCharacterConvertMatOfResidualRecord;
};