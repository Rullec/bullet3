#pragma once
#include "btBulletDynamicsCommon.h"
#include "btGenUtil/BulletUtil.h"
#include <memory>

class btGenRigidBody;
class btGenContactSolver;
class cRobotModelDynamics;
class btGenContactForce;
class btGenConstraintGeneralizedForce;
class btGenCollisionDispatcher;
class btGenControllerBase;
class btGenContactAwareController;
class btTraj;
class btGenContactManager;
class btGeneralizeWorld
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    enum eContactResponseMode
    {
        NoMode,
        LCPMode,
        PenaltyMode,
        SequentialImpulseMode
    };

    // struct tParams
    // {
    // 	tParams();
    // 	std::string mSimulatorConfig;
    // tVector mGravity;
    // double rigid_damping;
    // double mb_damping1;
    // double mb_damping2;
    // double mb_scale;
    // bool mb_enable_rk4;
    // eContactResponseMode Mode;
    // std::string mLCPConfigPath;
    // };

    // cSimulator(const tParams& params);
    btGeneralizeWorld();
    // btGeneralizeWorld(const std::string& config_path);
    ~btGeneralizeWorld();
    void Init(const std::string &config_path);
    void AddObj(int n, const std::string &obj_type, bool perturb = false);
    void AddStaticBody(btCollisionObject *obj, double mass,
                       const std::string &name);
    void RemoveStaticBody(btCollisionObject *obj);
    void SetGravity(const tVector &g);
    tVector GetGravity() const;
    void AddGround(double height);
    const btCollisionObject *GetGround() const;
    void AddMultibody(const std::string &path);
    void AddMultibody(cRobotModelDynamics *model);
    cRobotModelDynamics *GetMultibody();
    void ApplyTestActiveForce(double dt);
    void ClearForce();
    void RemoveObj(int id);
    void StepSimulation(double dt);
    void Reset();
    // get & set method
    std::vector<btGenContactForce *> GetContactForces() const;
    std::vector<btPersistentManifold *> GetContactManifolds() const;
    btGenContactAwareController *GetContactAwareController();
    btDiscreteDynamicsWorld *GetInternalWorld();
    btBroadphaseInterface *GetBroadphase();
    btGenCollisionDispatcher *GetDispatcher();
    btDefaultCollisionConfiguration *GetConfiguration();
    eContactResponseMode GetContactResponseMode() const;
    // void SetEnableContactAwareControl();
    void AddController(const std::string &path);
    bool HasController() const;
    bool HasContactAwareController() const;
    btGenContactManager *GetContactManager() const;

protected:
    btDiscreteDynamicsWorld *mInternalWorld;
    btBroadphaseInterface *m_broadphase;
    btGenCollisionDispatcher *m_dispatcher;
    btDefaultCollisionConfiguration *m_collisionConfiguration;
    std::vector<btGenRigidBody *> mSimObjs;
    cRobotModelDynamics *mMultibody;
    btTraj *mGuideTraj;
    // btGenPDController* mPDController;
    double mTime;
    int mFrameId;
    tVector mGravity;
    double mRigidDamping;
    double mMBDamping1;
    double mMBDamping2;
    bool mMBZeroInitPose;
    bool mMBZeroInitPoseVel;
    // bool mMBEnableRk4;
    double mMBEpsDiagnoalMassMat;
    double mMBMaxVel;
    // bool mDebugThreeContactForces;
    // bool mMBEnableContactAwareLCP;

    // std::string mControllerConfig; //
    bool mEnablePeturb; // enable external perturb
    btCollisionObject *mGround;
    bool mMBEnableCollectFrameInfo; // collect frame info and save
    bool mEnablePauseWhenMaxVel;
    int mMBCollectFrameNum;
    double mMBScale;
    eContactResponseMode mContactMode;
    btGenContactSolver *mLCPContactSolver;
    btGenControllerBase *mCtrl; // character controller
    std::string mLCPConfigPath;
    std::vector<btPersistentManifold *> mManifolds;

    std::vector<btGenContactForce *> mContactForces; // all contact force
    std::vector<btGenConstraintGeneralizedForce *>
        mConstraintGenalizedForce; // multibody generalized force for joint limit
    std::vector<btGenConstraintGeneralizedForce *> mContactAwareControlForce;
    btGenContactManager *mContactManager;
    std::vector<btCollisionShape *> mCollisionShapeArray;
    void createRigidBody(double mass, const btTransform &startTransform,
                         btCollisionShape *shape, const std::string &name,
                         const btVector4 &color = btVector4(1, 0, 0, 1));
    // void createRigidBodyNew(double mass, const btTransform& startTransform,
    // btCollisionShape* shape, const std::string& name, const btVector4& color
    // = btVector4(1, 0, 0, 1));
    btBoxShape *createBoxShape(const btVector3 &halfExtents);

    // 1. add gravity
    void ApplyGravity();

    // 1.1 update the contact aware LCP controller
    void UpdateController(double dt);

    // 2. collision detect
    void CollisionDetect();

    // 3. collision respose
    void CollisionResponse(double dt);
    void CollisionResponsePenalty(double dt);
    void CollisionResponseLCP(double dt);
    void CollisionResposeSI(double dt);
    void PushStatePreCollision();
    void PopStatePostColliison();
    void Update(double dt);
    // void UpdateTransform(double dt);
    void UpdateVelocityInternal(double dt);
    void UpdateVelocityInternalWithoutCoriolis(double dt);
    void PostUpdate(double dt);

    // sim record
    struct tFrameInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        int frame_id;
        double timestep;
        tVectorXd q, qdot,
            qddot; // generalized displacement, generalized vel, gen accel
        tVectorXd Q, residual; //  gen force, lagrangian residual
        tMatrixXd mass_mat, coriolis_mat, damping_mat, mass_mat_inv;

        tEigenArr<tVector> force_array, torque_array;
    };
    tEigenArr<tFrameInfo> mFrameInfo;
    void CollectFrameInfo(double dt);
    void WriteFrameInfo(const std::string &path);
    void RecordMBContactForce();
    // void InitGuideTraj();
    // void ApplyGuideAction();
    // void CheckGuideTraj();
};