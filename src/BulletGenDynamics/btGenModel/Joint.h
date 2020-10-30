#ifndef ROBOT_JOINT_H
#define ROBOT_JOINT_H
#include "BaseObject.h"
#include "ModelEigenUtils.h"
#include <vector>

class Joint : public BaseObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    explicit Joint(const BaseObjectParams &param);
    explicit Joint(const BaseObjectJsonParam &param);

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

    // =============First order derivative=============
    void ComputeLocalFirstDeriveMatrix(); // dRxdqx, dRy/dqy, dRz/dqz
    virtual void
    ComputeLocalTransformFirstDerive(); // T = Rz * Ry * Rx, calculate dT/dqx, dT/dqy, dT/dqz
    void ComputeGlobalTransformFirstDerive(); //

    // =============Second order derivative=============
    void ComputeLocalSecondDeriveMatrix();
    virtual void ComputeLocalTransformSecondDerive();
    void ComputeGlobalTransformSecondDerive();

    // =============Third order derivative=============
    void ComputeLocalThirdDeriveMatrix();
    virtual void ComputeLocalTransformThirdDerive();
    void ComputeGlobalTransformThirdDerive();

    virtual void GetRotations(tMatrix3d &m);

    void ComputeJacobiByGivenPointTotalDOF(tMatrixXd &j,
                                           const tVector &p) const override;
    void ComputeJacobiByGivenPoint(tMatrixXd &j,
                                   const tVector &p) const override;
    void ComputeHessianByGivenPoint(EIGEN_V_MATXD &ms,
                                    const tVector &p) const override;
    void ComputeJKv_dot(const tVectorXd &q_dot, const tVector3d &p) override;
    void ComputeJKw_dot(const tVectorXd &q_dot) override;

    void ComputeLocalTransform() override;
    void ComputeGlobalTransform() override;

    virtual const tMatrix &GetMWQQ(int i, int j) const override;
    virtual const tMatrix &GetMWQQQ(int i, int j, int k) const override;
    const tMatrix &GetMTQ(int i) const;
    const tMatrix &GetMTQQ(int ibb, int j) const;
    const tMatrix &GetMTQQQ(int i, int j, int k) const;
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

    EIGEN_V_tMatrixD
        r_m; // rotation/translation(only for root) matrices for the local freedoms in this joint, size = freeomds
    EIGEN_V_tMatrixD r_m_first_deriv;
    EIGEN_V_tMatrixD r_m_second_deriv;
    EIGEN_V_tMatrixD r_m_third_deriv;

    // ===========second derive==============
    EIGEN_VV_tMatrixD mTqq;
    EIGEN_VV_tMatrixD mWqq;
    // ======================================

    // ===========third derive==============
    EIGEN_VVV_tMatrixD
        mTqqq; // T is local trans, dT3/dq3 save the lower triangle
    EIGEN_VVV_tMatrixD
        mWqqq; // W is world trans, dW3/dq3 save the lower triangle
    // ======================================

    // joint torque limit
    double mTorqueLim;

    // diff weight, used in the calculation of mimic reward
    double mDiffWeight;

    // the linear velocity and angular velocity of the COM of this joint in
    // world frame
    tVector3d mJointVel, mJointOmega;

    // method
    void ComputeLocalTransformThirdDeriveSpherical();
};

#endif // ROBOT_JOINT_H
