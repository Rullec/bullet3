#pragma once
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class cRobotModelDynamics;
class btTraj;
struct btGenFeature;
struct tFeatureArraySingleOrder;
/**
 * \brief				This class construct the feature vector in contact-aware controller
 */
class btGenFeatureArray
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    // btGenFeatureArray(const Json::Value& conf);
    btGenFeatureArray();
    void Init(const std::string &conf, cRobotModelDynamics *model,
              const tVector &g);
    void Reset();
    // void Eval(const int target_frame, tMatrixXd& H, tMatrixXd& E, tVectorXd&
    // f);
    void Eval(double dt, const tVectorXd &qddot_target,
              const tVectorXd &qdot_target, const tVectorXd &q_target,
              const tVectorXd &tau_target, tMatrixXd &H, tMatrixXd &E,
              tVectorXd &f);
    void CalcEnergy(const tVectorXd &control_force,
                    const tVectorXd &contact_force);
    void DebugAccelFeatureDFdtauIsZero(const tVectorXd &control_force,
                                       const tVectorXd &contact_force);
    void DebugVelFeatureDFdtauIsZero(const tVectorXd &control_force,
                                     const tVectorXd &contact_force);
    void DebugPosFeatureDFdtauIsZero(const tVectorXd &control_force,
                                     const tVectorXd &contact_force);
    void DebugTauFeatureDFdtauIsZero(const tVectorXd &control_force,
                                     const tVectorXd &contact_force);
    void DebugFullMinimiumIsTheSameAsTheRefTraj(const tVectorXd &control_force,
                                                const tVectorXd &contact_force);
    void VerifyTheSolutionIsLocalMin(const tVectorXd &control_force,
                                     const tVectorXd &contact_force);
    virtual ~btGenFeatureArray();

protected:
    double mCurTimestep;
    cRobotModelDynamics *mModel;
    // btTraj* mTraj;  // guided trajectory

    // different order of feature array is managed in (3) different structure.
    std::vector<tFeatureArraySingleOrder *> mFeatureArrays; //
    tVector mGravity;                                       // gravity accel
    tVectorXd mWeight_tau; // mW1/2/3: accel/vel/pos feature vector
                           // weight, mWeight_tau: ref tau weight

    tMatrixXd mN; // convert the underacuated tau to full-DOF tau
    tMatrixXd mConvertPosToY,
        mConvertPosToXZ; // two convert matrix whican can convert [x, y, z] to
                         // "Y" or "X,Z"

    // record the target accel which has been set in the current frame
    // used in the calculation of energy term
    tVectorXd mTargetAccel, mTargetVel, mTargetPos;
    tVectorXd mTargetTau;

    tMatrixXd mH_prev, mE_prev;
    tVectorXd mf_prev;
    //--------------methods
    void InitWeightTau(const std::string &conf);
    void InitFeature(const std::string &conf);
    // void InitRefFeature();

    // Convert the contact force to q vars
    void EvalConvertMatFromForceToqddot(tMatrixXd &D_tau, tMatrixXd &M_Q,
                                        tVectorXd &n_tau) const;
    void EvalConvertMatFromForceToqdot(tMatrixXd &D_tau, tMatrixXd &M_Q,
                                       tVectorXd &n_tau) const;
    void EvalConvertMatFromForceToq(tMatrixXd &D_tau, tMatrixXd &M_Q,
                                    tVectorXd &n_tau) const;

    // convert the q vars to feature
    void EvalConvertMatAndResidualFromqddotToFeature(tMatrixXd &convert,
                                                     tVectorXd &residual) const;
    void EvalConvertMatAndResidualFromqdotToFeature(tMatrixXd &convert,
                                                    tVectorXd &residual) const;
    void EvalConvertMatAndResidualFromqToFeature(tMatrixXd &convert,
                                                 tVectorXd &residual) const;

    // calculate the final H, E, f matrix/vector
    void EvalDynamicTerms(const tVectorXd &ref_accel_feature,
                          const tVectorXd &ref_vel_feature,
                          const tVectorXd &ref_pos_feature,
                          const tVectorXd &ref_tau, tMatrixXd &H, tMatrixXd &E,
                          tVectorXd &f) const;

    tVectorXd CalcTargetAccelFeature(const tVectorXd &qddot);
    tVectorXd CalcTargetVelFeature(const tVectorXd &qdot);
    tVectorXd CalcTargetPosFeature(const tVectorXd &q);
    // int GetSingleFeatureOffset(int feature_id) const;
    btGenFeature *CreateJointFeature(const std::string &joint_name) const;
    btGenFeature *CreateHeightFeature(const std::string &link_name) const;
    btGenFeature *CreateLocationFeature(const std::string &link_name) const;
    void PrintFeatureInfo() const;

    void TestdJvdq(tVectorXd &q, tVectorXd &qdot, tVectorXd &qddot,
                   int link_id);

    void AssertOnlyAccelJointFeature(const std::string &prefix);
    void EvalHEfForOnlyAccelJointFeature(const tMatrixXd &res_H,
                                         tMatrixXd &res_E, tVectorXd &res_f);
    double CalcAccelFeatureEnergy(const tVectorXd &control_force,
                                  const tVectorXd &contact_force);
};