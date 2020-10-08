#pragma once
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
class btGenFrameByFrameOptimizer;
class btCollisionObject;
class btGenContactAwareAdviser
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
    // btGenContactAwareAdviser(btGeneralizeWorld* world, const std::string&
    // guide_traj, double W, double Wm);
    btGenContactAwareAdviser(btGeneralizeWorld *world);
    ~btGenContactAwareAdviser();
    void Init(cRobotModelDynamics *mModel, const std::string &config_file);
    // int GetInternalFrameId() const;
    void SetTraj(const std::string &mRefTrajPath,
                 const std::string &mOutputTraj, bool enable_output = false);
    btTraj *GetRefTraj();
    void Update(double dt);
    void UpdateMultibodyVelocityDebug(double dt);
    void UpdateMultibodyVelocityAndTransformDebug(double dt);
    tVectorXd CalcControlForce(const tVectorXd &Q_contact);
    tVectorXd GetPrevControlForce();
    tVectorXd CalcLCPResidual(double dt) const;
    tMatrixXd CalcLCPPartBPrefix() const;
    bool IsEnd();
    void Reset();
    btGenFrameByFrameOptimizer *GetFBFOptimizer();
    void SetBulletGUIHelperInterface(struct GUIHelperInterface *inter);
    // void GetAdviseInfo(tMatrixXd& C, tMatrixXd& D, tMatrixXd& E, tMatrixXd&
    // N, tVectorXd& b);

protected:
    double mCurdt;
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
    Json::Value mFrameByFrameConfig;
    bool mOutputControlDiff;
    bool mEnableSyncTrajPeriodly; // sync the ref traj to the simulation
                                  // character periodly
    bool mEnableOnlyFBFControl;   // use the contact force and control force
                                  // calculated by the FBF optimzier, apply them
                                  // to the model directly and check the result
    bool mEnableRefTrajDelayedUpdate; // when the ref traj hasn't been exetucted
                                      // perfectly, stop the forwardness of the
                                      // ref traj and make the control target
                                      // stay at the current frame
    bool
        mEnableDrawContactPointsInBulletGUIAdviser; // enable drawing contact points
    int mSyncTrajPeriod; // the sync period speicifed by config file
    cRobotModelDynamics *mModel;
    btGeneralizeWorld *mWorld;
    btGenFeatureArray *mFeatureVector;
    btGenFrameByFrameOptimizer *mFBFOptimizer;
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
    void PreUpdate(double dt);
    void PostUpdate();
    void ReadConfig(const std::string &config);
    void ResolveActiveForce();
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
    void DrawPoint(const tVector3d &pos, double r = 0.05);
};