#pragma once
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class btTraj;
class btGeneralizeWorld;
class cRobotModelDynamics;
class QuadProgQPSolver;
class btCharContactPt;
class btGenFrameByFrameConstraint;
class btGenFrameByFrameEnergyTerm;
enum eContactStatus
{
    INVALID_CONTACT_STATUS,
    SLIDING,
    STATIC,
    BREAKAGE
};
class btGenFrameByFrameOptimizer
{
public:
    btGenFrameByFrameOptimizer();
    ~btGenFrameByFrameOptimizer();
    void Init(btGeneralizeWorld *mWorld, const Json::Value &conf);
    void SetTraj(btTraj *traj_);
    void SetCoef(const Json::Value &conf);
    void CalcTarget(double dt, int target_frame_id, tVectorXd &tilde_qddot,
                    tVectorXd &tilde_qdot, tVectorXd &tilde_q,
                    tVectorXd &tilde_tau);
    void Reset();

protected:
    // ---------methods
    void ParseConfig(const Json::Value &conf);
    void InitModelInfo();
    void InitQPSolver();
    void CalcContactStatus();
    void CalcSolutionVector();
    void CalcEnergyTerms();
    void CalcConstraints();
    void Solve(tVectorXd &tilde_qddot, tVectorXd &tilde_qdot,
               tVectorXd &tilde_q, tVectorXd &tilde_tau);
    void CalcTargetInternal(const tVectorXd &solution, tVectorXd &qddot,
                            tVectorXd &qdot, tVectorXd &q, tVectorXd &tau);
    void ClearContactPoints();

    void AddSlidingConstraint(btCharContactPt *pt);
    void AddStaticConstraint(btCharContactPt *pt);
    void AddBreakageConstraint(btCharContactPt *pt);
    void AddFixStaticContactPointConstraint();
    void AddContactForceLimitConstraint();
    void AddDynamicEnergyTerm();
    void AddDynamicEnergyTermPos();
    void AddDynamicEnergyTermVel();
    void AddDynamicEnergyTermAccel();
    void AddDynamicEnergyTermMinAccel();
    void AddDynamicConstraint();
    void AddMinTauEnergyTerm();
    void AddMinContactForceEnergyTerm();
    void AddTauCloseToOriginEnergyTerm();
    void AddContactForceCloseToOriginEnergyTerm();
    void AddEndEffectorPosEnergyTerm();
    void AddEndEffectorVelEnergyTerm();
    void AddEndEffectorOrientationEnergyTerm();
    void AddRootPosEnergyTerm();
    void AddRootVelEnergyTerm();
    void AddRootOrientationEnergyTerm();
    void AddLinkPosEnergyTerm(int link_id, double coef,
                              const tVector3d &target_pos);
    void AddLinkPosEnergyTermIgnoreRoot(
        int link_id, double coef,
        const tVector3d &target_link_local_pos_wrt_root);
    void AddLinkVelEnergyTerm(int link_id, double coef,
                              const tVector3d &target_vel);
    void AddLinkOrientationEnergyTerm(int link_id, double coef,
                                      const tMatrix3d &target_orientation);
    void CalcContactConvertMat(btCharContactPt *contact,
                               tMatrixXd &convert_mat);
    int GetSolutionSizeByContactStatus(eContactStatus status);

    // ---------vars
    int mCurFrameId;
    double mdt;
    QuadProgQPSolver *mQPSolver;
    cRobotModelDynamics *mModel;
    btTraj *mTraj;
    btGeneralizeWorld *mWorld;
    std::vector<btCharContactPt *> mContactPoints;
    std::vector<int> mContactSolOffset; // record the offset of each contact
                                        // point in solution vector
    std::vector<int> mContactSolSize;   // record the occupied "size" w.r.t each
                                        // contact pt in solution vector

    // optimization vars
    // bool mEnableHardConstraintForDynamics;
    // bool mUseNativeRefTarget;
    double mDynamicPosEnergyCoeff;
    double mDynamicVelEnergyCoeff;
    double mDynamicAccelEnergyCoeff;
    double mDynamicMinAccelEnergyCoeff;
    double mControlForceCoef;
    double mContactForceCoef;
    // double mControlForceCloseToOriginCoef;
    // double mContactForceCloseToOriginCoef;
    double mEndEffectorPosCoef;
    double mEndEffectorVelCoef;
    double mEndEffectorOrientationCoef;
    double mRootPosCoef;
    double mRootVelCoef;
    double mRootOrientationCoef; // control the orientation of root link

    bool mEnableContactForceLimit; // limit the contact force in [-a, a]
    double mContactForceLimit;     // the limitation of contact force

    bool mEnableContactConstraint; // add contact constraint in this
                                   // frame-by-frame constraint
    bool mEnableContactReduction;  // reduct the contact points at a link to a
                                   // single point (average)

    bool mEnableFixStaticContactPoint;  // add hard/fix constraint for static
                                        // contact
    bool mIgnoreRootPosInDynamicEnergy; // ignore the root dof in Pos/Vel/Accel
                                        // energy term
    bool mIgnoreRootBiasInEndEffectorControl;
    int mContactSolutionSize; // the size of contact force in result vector
    int mCtrlSolutionSize;    // the size of active ctrl froce in result vector
    int mTotalSolutionSize;   // total solution size

    tMatrixXd A, // E = x^T A x + x^T b
        Aeq, Aineq;
    tVectorXd b, beq, bineq;
    btGenFrameByFrameConstraint *mConstraint;
    btGenFrameByFrameEnergyTerm *mEnergyTerm;

    // model buffer vars
    int num_of_freedom;
    int num_of_underactuated_freedom;
};