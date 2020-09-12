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
    // btGenContactAwareAdviser(btGeneralizeWorld* world, const std::string&
    // guide_traj, double W, double Wm);
    btGenContactAwareAdviser(btGeneralizeWorld *world,
                             const std::string &config);
    ~btGenContactAwareAdviser();
    void Init(cRobotModelDynamics *model);
    void Update(double dt);
    void UpdateMultibodyVelocityDebug(double dt);
    void UpdateMultibodyVelocityAndTransformDebug(double dt);
    tVectorXd CalcControlForce(const tVectorXd &Q_contact);
    tVectorXd CalcLCPResidual(double dt) const;
    tMatrixXd CalcLCPPartBPrefix() const;
    // void GetAdviseInfo(tMatrixXd& C, tMatrixXd& D, tMatrixXd& E, tMatrixXd&
    // N, tVectorXd& b);

protected:
    double mCurdt;
    int mInternalFrameId;
    bool mOutputControlDiff;
    bool mEnableFrameByFrameCtrl;
    std::string mOutputControlDiffFile;
    bool mDrawReferenceTrajCharacter;
    bool mDrawTargetFBFCharacter;
    int mMaxFrame;
    int mStartFrame;
    std::string mFeatureVectorFile;
    std::string mTrajPath;
    Json::Value mCtrlConf;
    bool mResolveControlToruqe;
    cRobotModelDynamics *mModel;
    btGeneralizeWorld *mWorld;
    btGenFeatureArray *mFeatureVector;
    btGenFrameByFrameOptimizer *mFBFOptimizer;
    bool mEnableRecordCtrledTraj;
    std::string mSavedTrajPath;
    btTraj *mSavedTraj;
    btTraj *mRefTraj;
    cRobotModelDynamics *mRefModel;

    int num_of_freedom;               // model dof
    int num_of_underactuated_freedom; // model dof - 6
    tMatrixXd mN; // convert underactuated tau to full dof tau
    tMatrixXd mH,
        mE;       // convert matrices, please check the Note for more details
    tVectorXd mf; // convert vector, please check the Note for more details
    tVectorXd mTargetAccel, mTargetTau;
    // ----------------------- methods
    void ReadConfig(const std::string &config);
    void ResolveActiveForce();
    void LoadTraj(const std::string &traj);
    void GetTargetInfo(double dt, tVectorXd &qddot_target,
                       tVectorXd &tau_target);
    void CreateFeatureVector();
    void PostProcess();
    bool IsEnd();
    void CreateRefChar();
    void UpdateRefChar();
    void RecordTraj();
};