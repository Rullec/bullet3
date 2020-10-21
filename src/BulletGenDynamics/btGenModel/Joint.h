#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H
#include "BaseObject.h"
#include "ModelEigenUtils.h"
#include <vector>

class Joint : public BaseObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    explicit Joint(BaseObjectParams &param);
    explicit Joint(BaseObjectJsonParam &param);

    void Tell() override;
    Freedom *AddFreedom(Freedom &f) override;
    int GetNumOfFreedom() override;
    void SetFreedomValue(int id, double v) override;
    void GetFreedomValue(int id, double &v) override;

    void SetFreedomValue(std::vector<double> &v) override;
    void GetFreedomValue(std::vector<double> &v) override;

    void CleanGradient() override;

    Freedom *GetFreedoms(int order) override;
    Freedom *GetFreedomByAxis(tVector3d axis, int type = REVOLUTE) override;

    void InitTerms() override;
    void InitMatrix();

    bool IsJoint() const override { return true; }
    void UpdateState(bool compute_gradient) override;
    void UpdateMatrix();
    void ComputeLocalFirstDeriveMatrix();
    void ComputeLocalSecondDeriveMatrix();
    virtual void GetRotations(tMatrix3d &m);
    virtual void ComputeTransformFirstDerive();
    void ComputeGlobalTransformFirstDerive();

    virtual void ComputeLocalTransformSecondDerive();
    void ComputeGlobalTransformSecondDerive();

    void ComputeJacobiByGivenPointTotalDOF(tMatrixXd &j,
                                           const tVector &p) const override;
    void ComputeJacobiByGivenPoint(tMatrixXd &j,
                                   const tVector &p) const override;
    void ComputeHessianByGivenPoint(EIGEN_V_MATXD &ms,
                                    const tVector &p) const override;
    void ComputeJKv_dot(tVectorXd &q_dot, tVector3d &p) override;
    void ComputeJKw_dot(tVectorXd &q_dot) override;

    void ComputeLocalTransform() override;
    void ComputeGlobalTransform() override;

    tMatrix &GetMWQQ(int i, int j) override;

    const tMatrixXd &GetJKDot() const override;

    void SetTorqueLim(double lim);
    double GetTorqueLim() const;
    void SetDiffWeight(double lim);
    double GetDiffWeight() const;

    void SetJointVel(const tVector3d &vel_);
    void SetJointOmega(const tVector3d &omega_);
    tVector3d GetJointVel() const;
    tVector3d GetJointOmega() const;

protected:
    std::vector<Freedom>
        freedoms; // self freedom. For revolute it is 1, for spherical it is 3.

    EIGEN_V_tMatrixD r_m;   // rotation/translation(only for root) matrices for the local freedoms in this joint, size = freeomds
    EIGEN_V_tMatrixD r_m_first_deriv;
    EIGEN_V_tMatrixD r_m_second_deriv;

    // ===========second derive==============
    EIGEN_VV_tMatrixD mTqq;
    EIGEN_VV_tMatrixD mWqq;
    // ======================================

    // joint torque limit
    double mTorqueLim;

    // diff weight, used in the calculation of mimic reward
    double mDiffWeight;

    // the linear velocity and angular velocity of the COM of this joint in
    // world frame
    tVector3d mJointVel, mJointOmega;
};

#endif // ROBOT_JOINT_H
