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
    int GetInternalFrameId() const;
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
    // void GetAdviseInfo(tMatrixXd& C, tMatrixXd& D, tMatrixXd& E, tMatrixXd&
    // N, tVectorXd& b);

protected:
    double mCurdt;
    bool mHasRefTraj;
    int mInternalFrameId;
    bool mDrawReferenceTrajCharacter;
    bool mDrawTargetFBFCharacter;
    int mMaxFrame;
    int mStartFrame;
    std::string mFeatureVectorFile;
    tVectorXd mCtrlForce;
    bool mResolveControlToruqe;
    Json::Value mFrameByFrameConfig;
    bool mOutputControlDiff;
    bool mEnableSyncTrajPeriodly;
    int mSyncTrajPeriod;
    cRobotModelDynamics *mModel;
    btGeneralizeWorld *mWorld;
    btGenFeatureArray *mFeatureVector;
    btGenFrameByFrameOptimizer *mFBFOptimizer;
    btTraj *mOutputTraj;
    btTraj *mRefTraj;
    std::string mRefTrajPath;
    std::string mOutputTrajPath;
    bool mEnableOutput;
    cRobotModelDynamics *mRefModel;

    int num_of_freedom;               // model dof
    int num_of_underactuated_freedom; // model dof - 6
    tMatrixXd mN; // convert underactuated tau to full dof tau
    tMatrixXd mH,
        mE;       // convert matrices, please check the Note for more details
    tVectorXd mf; // convert vector, please check the Note for more details
    tVectorXd mTargetAccel, mTargetVel, mTargetPos, mTargetTau;

    // ----------------------- methods
    void ReadConfig(const std::string &config);
    void ResolveActiveForce();
    void LoadTraj(const std::string &traj);
    void GetTargetInfo(double dt, tVectorXd &qddot_target,
                       tVectorXd &qdot_target, tVectorXd &q_target,
                       tVectorXd &tau_target);
    void CreateFeatureVector();
    void PostProcess();
    void CreateRefChar();
    void UpdateRefChar();
    void RecordTraj();
};