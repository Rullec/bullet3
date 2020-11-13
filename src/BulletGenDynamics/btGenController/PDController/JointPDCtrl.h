#pragma once
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

/**
 * \brief       PD Controller for a single joint
*/
class Joint;
class btGenJointPDCtrl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenJointPDCtrl(Joint *, double kp, double kd, double force_lim, bool use_world_coord);
    void SetKp(double Kp);
    void SetKd(double Kd);
    double GetKp(double Kp) const;
    double GetKd(double Kd) const;
    bool GetUseWorldCoord() const;
    void SetTargetTheta(const tVectorXd &theta);
    void SetTargetVel(const tVectorXd &vel);
    tVectorXd GetTargetTheta() const;
    tVectorXd GetTargetVel() const;
    Joint *GetJoint() const;
    tVector CalcControlForce() const;
    int GetCtrlDims() const;

protected:
    Joint *mJoint;
    double mKp, mKd;
    double mForceLim;
    const bool mUseWorldCoord;
    tVectorXd
        mTargetTheta; // PD target, expressed as the part of q (gen coordinates), euler angles x-y-z
    tVectorXd
        mTargetVel; // PD target, expressed as the part of qdot (gen vel), d(euler angles)/dq, xdot-ydot-zdot
    void CheckCtrlDims(const tVectorXd &var, std::string prefix) const;
    void ControlForceNone(tVector &force) const;
    void ControlForceRevolute(tVector &force) const;
    void ControlForceSpherical(tVector &force) const;
    void ControlForceFixed(tVector &force) const;
};