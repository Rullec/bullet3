#pragma once
#include "RobotModel.h"

class btDynamicsWorld;
class btGenContactAwareController;
class cRobotModelDynamics : public cRobotModel
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // cRobotModelDynamics(const char* model_file, double scale, int type);
    cRobotModelDynamics();
    virtual ~cRobotModelDynamics();
    void Init(const char *model_file, double scale, int type);

    // ----------------------simulation APIs----------------------
    void InitSimVars(btDynamicsWorld *world, bool zero_pose, bool zero_pose_vel,
                     bool enable_collision = true);
    // void InitSimVars(btDiscreteDynamicsWorld* world, bool zero_pose, bool
    // zero_pose_vel);
    virtual void SetqAndqdot(const tVectorXd &q, const tVectorXd &qdot);
    virtual void Setqdot(const tVectorXd &qdot);
    void SetMassMatEps(double eps);
    void ApplyGravity(const tVector &g); // apply force
    tVectorXd CalcGenGravity(
        const tVector &g) const; // calculate the generalized gravity, sometimes
                                 // useful in external calculation
    tMatrixXd CalcdGenGravitydq(const tVector &g) const;
    void TestdGenGravitydq(const tVector &g);

    void ApplyForce(int link_id, const tVector &f, const tVector &applied_pos);
    void ApplyForce3d(int link_id, const tVector3d &f,
                      const tVector3d &applied_pos);
    void ApplyGeneralizedForce(int dof_id, double value);
    void ApplyLinkTorque(int link_id, const tVector &torque);
    void ApplyJointTorque(int joint_id, const tVector &torque);
    tVectorXd GetGeneralizedForce();
    tVectorXd DebugGetGeneralizedForce();
    btGenRobotCollider *GetLinkCollider(int link_id);
    void ClearForce();
    tVectorXd Getqddot();
    void UpdateVelocityAndTransform(double dt);
    void UpdateVelocity(double dt, bool verbose = false);
    void UpdateVelocityWithoutCoriolis(double dt);
    void UpdateTransform(double dt);
    // void UpdateRK4(double dt);
    void SyncToBullet();
    bool IsGeneralizedMaxVel(double max_vel = 100.0) const;
    double GetMaxVelThreshold() const;
    bool IsCartesianMaxVel(double max_vel = 100.0) const;
    double GetMaxVel() const;
    void PushState(const std::string &tag, bool only_vel_and_force = false);
    void PopState(const std::string &tag, bool only_vel_and_force = false);
    void SetDampingCoeff(double, double);
    // void SetAngleClamp(bool);
    void SetMaxVel(double);
    const tMatrixXd &GetDampingMatrix() const { return mDampingMatrix; }
    void TestJacobian();
    void TestSecondJacobian();
    void TestThirdJacobian();
    tMatrixXd GetInvMassMatrix();
    void GetEffectInfo(tEigenArr<tVector> &force_array,
                       tEigenArr<tVector> &torque_array);
    void SetContactAwareController(btGenContactAwareController *ptr);
    btGenContactAwareController *GetContactAwareController() const;
    bool GetEnableContactAwareController() const;
    void SetEnableContactAwareController(bool);
    bool GetCollisionEnabled() const;
protected:
    // -------------------------simulation status-----------------------
    std::vector<btGenRobotCollider *>
        mColliders; // used in bullet collision detection for multibody
                    // structure
    // std::vector<btCollisionShape*> mColShapes;   // used in bullet collision
    // detection for multibody structure
    tEigenArr<tVector> mLinkForces,
        mLinkTorques; // the external force & torque applied on each link
    // tEigenArr<tVector> link_vel, link_omega;			// the lin & ang vel with
    // respect to each link (COM) in world frame
    tVectorXd mGenForce; // the external generalized force applied on multibody
                         // (usually used in joint limit constraint)
    tMatrixXd mDampingMatrix; // damping matrix, has the same shape with
                              // mass_mat and coriolis_mat
    struct tStateRecord
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        tStateRecord();
        bool only_vel_and_force_recorded;
        tVectorXd q, qdot;
        tVectorXd generalized_force;
        tEigenArr<tVector> link_forces, link_torques;
        tMatrixXd mass_matrix;
        tMatrixXd coriolis_matrix;
        tMatrixXd damping_matrix;
        bool compute_2rd_derive;
        bool compute_3ed_derive;
    };

    const int mStackCapacity = 10;
    tEigenArr<std::pair<std::string, tStateRecord *>> mStateStack;
    bool mEnableContactAwareController;
    btGenContactAwareController *mController;
    // --------------------------configuration---------------------------
    bool mEnableCollision;
    double mDampingCoef1,
        mDampingCoef2;           // damping coeffs used in Rayleigh damping
    bool mEnableEulerAngleClamp; // always clamp the euler angle to [-pi, pi]
    double mEpsDiagnoal; // add epsilon on the diagonal of mass matrix, in order
                         // to refine it.
    double mMaxVel;      // the max velocity of generalized coordinates
    // btDynamicsWorld * mBtWorld;
    // member methods
    // void UpdateRK4InternalUpdate(tVectorXd& q, tVectorXd& qdot, tVectorXd&
    // Q);
    void UpdateCartesianVelocity();
    void ComputeMassMatrix();
    void ComputeDampingMatrix();
    void TestRotationChar();
    void TestLinkddJvddq(int id);
    void TestLinkddJwddq(int id);
};
