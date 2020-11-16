#pragma once
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

/**
 * \brief       PD Controller for a single joint
*/
class Joint;
class cRobotModelDynamics;
class btGenJointPDCtrl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenJointPDCtrl(cRobotModelDynamics *, Joint *, double kp, double kd,
                     double force_lim, bool use_world_coord);
    void SetKp(double Kp);
    void SetKd(double Kd);
    double GetKp() const;
    double GetKd() const;
    bool GetUseWorldCoord() const;
    void SetUseWorldCoord(bool val);
    Joint *GetJoint() const;
    tVector CalcControlForce(const tVectorXd &q, const tVectorXd &qdot) const;
    int GetCtrlDims() const;
    void BuildTargetPose(tVectorXd &q);
    void BuildTargetVel(tVectorXd &qdot);

protected:
    cRobotModelDynamics *mModel;
    Joint *mJoint;
    double mKp, mKd;
    double mForceLim;
    bool mUseWorldCoord;
    void CheckCtrlDims(const tVectorXd &var, std::string prefix) const;
    void ControlForceNone(tVector &force, const tVectorXd &local_target_theta,
                          const tVectorXd &local_target_vel) const;
    void ControlForceRevolute(tVector &force,
                              const tVectorXd &local_target_theta,
                              const tVectorXd &local_target_vel) const;
    void ControlForceSpherical(tVector &force,
                               const tVectorXd &local_target_theta,
                               const tVectorXd &local_target_vel) const;
    void ControlForceFixed(tVector &force, const tVectorXd &local_target_theta,
                           const tVectorXd &local_target_vel) const;
    void CalcLocalControlTarget(tVectorXd &mTargetThetaWorldOrLocal,
                                const tQuaternion &target_global_quad) const;
};