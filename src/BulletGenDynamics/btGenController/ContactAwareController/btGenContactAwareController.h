#pragma once
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
/**
 * \brief               In contact-aware character control, the LCP problem is
 * resposible to both the contact force and the control forces This class tries
 * to provide a motion guidence (the acceleration of character) in current frame
 */
struct btTraj;
class btGenFeatureArray;
class btGeneralizeWorld;
class cRobotModelDynamics;
class btGenFBFTargetCalculator;
class btCollisionObject;
class btGenTargetCalculator;
class btGenContactAwareController : public btGenControllerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // struct tParams
    // {
    //     tParams();
    //     btGeneralizeWorld *mWorld;

    //     std::string mFeatureVectorFile;
    //     bool mResolveControlTau;
    //     bool mbtDrawRefTargetFBFCharacter;
    //     bool mbtDrawReferenceTrajCharacter;
    //     int mMaxFrame;
    //     int mStartFrame;
    //     double mFBFPosCoef, mFBFVelCoef, mFBFAccelCoef;
    // };
    // btGenContactAwareController(btGeneralizeWorld* world, const std::string&
    // guide_traj, double W, double Wm);
    btGenContactAwareController(btGeneralizeWorld *world);
    ~btGenContactAwareController();
    virtual void Init(cRobotModelDynamics *mModel,
                      const std::string &config_file) override;
    // int GetInternalFrameId() const;
    void SetTraj(const std::string &mRefTrajPath,
                 const std::string &mOutputTraj, bool enable_output = false);
    const btTraj *GetRefTraj() const;
    virtual void Update(double dt) override;
    void UpdateMultibodyVelocityDebug(double dt);
    void UpdateMultibodyVelocityAndTransformDebug(double dt);
    tVectorXd CalcControlForce(const tVectorXd &Q_contact, bool verbose = true);
    tVectorXd GetPrevControlForce();
    tVectorXd CalcLCPResidual(double dt) const;
    tMatrixXd CalcLCPPartBPrefix() const;
    bool IsEnd();
    virtual void Reset() override;
    btGenTargetCalculator *GetTargetCalculator();
    void SetBulletGUIHelperInterface(struct GUIHelperInterface *inter);
    int GetRefFrameId() const { return mRefFrameId; }
    std::string GetSupposedContactInfo();
    // void GetAdviseInfo(tMatrixXd& C, tMatrixXd& D, tMatrixXd& E, tMatrixXd&
    // N, tVectorXd& b);

protected:
    bool mHasRefTraj;
    int mSimFrameId; // simulation id, +=1 per frame
    int mRefFrameId; // the frame id of target in ref traj
    bool mDrawReferenceTrajCharacter;
    bool mDrawTargetFBFCharacter;
    int mMaxFrame;
    int mStartFrame;
    std::string mFeatureVectorFile;
    tVectorXd mCtrlForce;
    bool mResolveControlToruqe; // given the mocap data (ref traj), re solve the
                                // control torque
    bool
        mEnableMigrateContactInfo; // move the contact points in the ref traj to the given configuration
    std::string mMigratecontactInfoPath; // given an ideal contact point file
    std::string mTargetCalculatorConfigFile;
    bool mOutputControlDiff;
    bool mEnableSyncTrajPeriodly;     // sync the ref traj to the simulation
                                      // character periodly
    bool mEnableOnlyTargetController; // use the contact force and control force
        // calculated by the FBF optimzier, apply them
        // to the model directly and check the result
    bool mEnableRefTrajDelayedUpdate; // when the ref traj hasn't been exetucted
                                      // perfectly, stop the forwardness of the
                                      // ref traj and make the control target
                                      // stay at the current frame
    bool
        mEnableDrawContactPointsInBulletGUIController; // enable drawing contact points
    bool
        mEnableDrawRefTrajContactPoints; // enable drawing contact points in ref traj
    int mSyncTrajPeriod; // the sync period speicifed by config file

    btGenFeatureArray *mFeatureVector;
    btGenTargetCalculator *mTargetCalculator;
    bool mEnableStateSave;
    bool mEnableInitStateLoad;
    std::string mInitStateFile;

    std::string mStateSaveDir;
    btTraj *mOutputTraj;
    btTraj *mRefTraj;
    std::string mRefTrajPath;
    std::string mOutputTrajPath;
    bool mEnableOutput;
    cRobotModelDynamics *mRefTrajModel;
    cRobotModelDynamics *mFBFTrajModel;

    int num_of_freedom;               // model dof
    int num_of_underactuated_freedom; // model dof - 6
    tMatrixXd mN; // convert underactuated tau to full dof tau
    tMatrixXd mH,
        mE;       // convert matrices, please check the Note for more details
    tVectorXd mf; // convert vector, please check the Note for more details
    tVectorXd mTargetAccel, mTargetVel, mTargetPos, mTargetTau;

    // draw contact points utils
    struct GUIHelperInterface *mBulletGUIHelper;
    std::vector<btCollisionObject *> mDrawPointsList;

    // ----------------------- methods

    btGenTargetCalculator *CreateTargetCalculator(const std::string path) const;
    void PreUpdate(double dt);
    void PostUpdate();
    void ReadConfig(const std::string &config);
    void ResolveActiveForce();
    void MigrateTrajContactByGivenInfo();
    void LoadTraj(const std::string &traj);
    void FetchControlTarget(double dt, tVectorXd &qddot_target,
                            tVectorXd &qdot_target, tVectorXd &q_target,
                            tVectorXd &tau_target);
    void CreateFeatureVector();
    void PostProcess();
    void CreateRefChar();
    void UpdateDrawingChar();
    void RecordTraj();
    void SaveCurrentState();
    void LoadInitState();

    void UpdateReferenceTraj();
    void ClearDrawPoints();
    void DrawContactPoints();
    void DrawRefTrajContactPoints();
    void DrawPoint(const tVector3d &pos, double r = 0.05);
};