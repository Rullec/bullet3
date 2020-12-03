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
    tVectorXd CalcJointTargetPose(const tVectorXd &q) const;
    tVectorXd CalcJointTargetVel(const tVectorXd &qdot) const;
    double GetForceLim() const;

protected:
    cRobotModelDynamics *mModel;
    Joint *mJoint;
    double mKp, mKd;
    double mForceLim;
    bool mUseWorldCoord;
    void CheckCtrlDims(const tVectorXd &var, std::string prefix) const;
    void ControlForceNone(tVector &force, const tVectorXd &local_target_theta,
                          const tVectorXd &local_target_vel) const;
    void ControlForceBipedalNone(tVector &force,
                                 const tVectorXd &local_target_theta,
                                 const tVectorXd &local_target_vel) const;
    void ControlForceLimitNone(tVector &force,
                               const tVectorXd &local_target_theta,
                               const tVectorXd &local_target_vel) const;
    void ControlForceRevolute(tVector &force,
                              const tVectorXd &local_target_theta,
                              const tVectorXd &local_target_vel) const;
    void ControlForceSpherical(tVector &force,
                               const tVectorXd &local_target_theta,
                               const tVectorXd &local_target_vel) const;
    void ControlForceFixed(tVector &force, const tVectorXd &local_target_theta,
                           const tVectorXd &local_target_vel) const;
    tVectorXd CalcLocalControlTargetByWorldTarget(
        const tQuaternion &target_global_quad) const;
    void DebugVerifyCtrlForceKp(const tVectorXd &local_target_theta,
                                const tVector &torque);
    void DebugVerifyCtrlForceKd(const tVectorXd &local_target_theta_dot,
                                const tVector &torque);
};