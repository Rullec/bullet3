#pragma once
#include "btBulletDynamicsCommon.h"
#include "btGenUtil/BulletUtil.h"
#include <memory>

class btGenRigidBody;
class btGenContactSolver;
class cRobotModelDynamics;
class btGenContactForce;
struct btGenConstraintGeneralizedForce;
class btGenCollisionDispatcher;
class btGenControllerBase;
class btGenContactAwareController;
struct btTraj;
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

    enum eIntegrationScheme
    {
        OLD_SEMI_IMPLICIT_SCHEME,
        NEW_SEMI_IMPLICIT_SCHEME,
        NUM_OF_INTEGRATION_SCHEME
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
    void TestDxnextDCtrlForce();
    // get & set method
    std::vector<btGenContactForce *> GetContactForces() const;
    std::vector<btPersistentManifold *> GetContactManifolds() const;
    btGenContactAwareController *GetContactAwareController();
    btGenControllerBase *GetController();
    btDiscreteDynamicsWorld *GetInternalWorld();
    btBroadphaseInterface *GetBroadphase();
    int GetNumOfRigidBody() const;
    btGenRigidBody *GetRigidBodyById(int id);
    btGenRigidBody *GetRigidBodyByName(std::string name);
    btGenCollisionDispatcher *GetDispatcher();
    btDefaultCollisionConfiguration *GetConfiguration();
    eContactResponseMode GetContactResponseMode() const;
    // void SetEnableContactAwareControl();
    void AddController(const std::string &path);
    bool HasController() const;
    bool HasContactAwareController() const;
    btGenContactManager *GetContactManager() const;
    eIntegrationScheme GetIntegrationScheme() const;
    static eIntegrationScheme BuildIntegrationScheme(const std::string str);
    static std::string BuildIntegrationSchemeStr(eIntegrationScheme scheme);
    btCollisionShape *GetSphereCollisionShape(double radius);
    btCollisionShape *GetBoxCollisionShape(const tVector3d &half_extends);
    btCollisionShape *GetCapsuleCollisionShape(double radius, double height);
    void GetLastFrameGenCharqAndqdot(tVectorXd &q_pre,
                                     tVectorXd &qdot_pre) const;

    tMatrixXd GetDQcDu() const;
    tMatrixXd GetDxnextDQc_u() const;
    tMatrixXd GetDxnextDQGDQGDxcur() const;
    tMatrixXd GetDxnextDxcur() const;

protected:
    double mCurdt;
    btDiscreteDynamicsWorld *mInternalWorld;
    btBroadphaseInterface *m_broadphase;
    btGenCollisionDispatcher *m_dispatcher;
    btDefaultCollisionConfiguration *m_collisionConfiguration;
    std::vector<btGenRigidBody *> mSimObjs;
    cRobotModelDynamics *mMultibody;
    btTraj *mGuideTraj;
    eIntegrationScheme mIntegrationScheme;
    // btGenPDController* mPDController;
    double mTime;
    int mFrameId;
    tVector mGravity;
    double mRigidDamping;
    double mMBDamping1;
    double mMBDamping2;
    std::string mMBInitPose;

    // test the gradient d(x_next)/d(ctrl_force), used in diff world
    bool mEnableTestGradientDxnextDctrlForce;
    bool mEnableTestGradientDxnextDctrlForceRandomCtrlForce;
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
    std::vector<std::pair<int, btCollisionShape *>> mCollisionShapeArray;
    void createRigidBody(double mass, const btTransform &startTransform,
                         btCollisionShape *shape, const std::string &name,
                         const btVector4 &color = btVector4(1, 0, 0, 1));
    // void createRigidBodyNew(double mass, const btTransform& startTransform,
    // btCollisionShape* shape, const std::string& name, const btVector4& color
    // = btVector4(1, 0, 0, 1));
    // btBoxShape *createBoxShape(const btVector3 &halfExtents);

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
    // void UpdateVelocityInternalWithoutCoriolis(double dt);
    void PostUpdate(double dt);
    void PreUpdate(double dt);

    // sim record
    struct tFrameInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        tFrameInfo();
        int frame_id;
        double timestep;
        tVectorXd q, qdot,
            qddot; // generalized displacement, generalized vel, gen accel
        tVectorXd Q, residual; //  gen force, lagrangian residual
        tMatrixXd mass_mat, coriolis_mat, damping_mat, mass_mat_inv;

        tEigenArr<tVector> force_array, torque_array;
    };
    tEigenArr<tFrameInfo> mFrameInfo;
    tFrameInfo mLastFrameInfo;
    void CollectFrameInfo(double dt);
    void WriteFrameInfo(const std::string &path);
    void RecordMBContactForce();

    // API in diffWorld
    void CalcDiffWorld();
    tMatrixXd CalcDQconsDu() const;
    tMatrixXd CalcDxnextDQc_Qu() const;
    tMatrixXd CalcDxnextDQGDQGDxcur() const;
    tMatrixXd CalcDxnextDxcur() const;
    // tMatrixXd CalcDxnextDxcur_noforce() const;
    tMatrixXd mDQconsDu;          // d(x_next)/d(ctrl_force), jacobian
    tMatrixXd mDxnextDQc_u;       // d(x_next)/d(Q_c), jacobian
    tMatrixXd mDxnextDQGDQGDxcur; // d(x_next)/d(QG) * d(QG)/d(xcur), jacobian
    tMatrixXd mDxnextDxcur;       // d(x_next)/d(x_cur)
    void TestAllDxnextDxcur();

    // tMatrixXd CalcDqnextDqcur() const;
    // tMatrixXd CalcDqnextDqdotcur() const;
    // tMatrixXd CalcDqdotnextDqcur() const;
    // tMatrixXd CalcDqdotnextDqdotcur() const;
    // tMatrixXd mDqnextDqcur, mDqnextDqdotcur, mDqdotnextDqcur,
    //     mDqdotnextDqdotcur;
    // void TestDqnextDqcur();
    // void TestDqdotnextDqcur();
    // void TestDqnextDqdotcur();
    // void TestDqdotnextDqdotcur();
};