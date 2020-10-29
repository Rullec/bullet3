#ifndef ROBOT_LINK_H
#define ROBOT_LINK_H

#include "BaseObject.h"

class Link : public BaseObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    explicit Link(const BaseObjectParams &param);
    explicit Link(const BaseObjectJsonParam &param);

    void Tell() override;

    void UpdateState(bool compute_gradient) override;
    void UpdateMeshMatrix() override;
    void InitTerms() override;
    bool IsJoint() const override { return false; }

    void UpdateMWQ();
    const tMatrix &GetMWQQ(int i, int j) const override;
    const tMatrix &GetMWQQQ(int i, int j, int k) const override;

    //===============For RigidBody Dynamics===============
    void ComputeMassMatrix() override;
    void ComputedMdq(tEigenArr<tMatrixXd> &dMdq);
    void ComputeJKv_dot(tVectorXd &q_dot, tVector3d &p) override;
    void ComputeJKw_dot(tVectorXd &q_dot) override;
    void ComputeDJkvdq(const tVector3d &p) override;
    void ComputeDJkwdq() override;
    void ComputeDDJkvddq(const tVector3d &p) override;
    void ComputeDDJkwdqq() override;
    //====================================================

    //===================For Simulation===================
    void SetLinkVel(const tVector3d &link_vel);
    void SetLinkOmega(const tVector3d &link_omega);
    tVector3d GetLinkVel() const;
    tVector3d GetLinkOmega() const;
    double GetLinkMaxLength() const;
    int GetColGroup() const;
    void SetColGroup(int);
    tMatrixXd GetTotalDofdJKv_dq(int target_dof_id) const;
    tMatrixXd GetTotalDofdJKw_dq(int target_dof_id) const;
    tMatrixXd GetddJKv_dqq(int i, int j) const;
    tMatrixXd GetddJKw_dqq(int i, int j) const;
    tMatrixXd GetTotalDofddJKv_dqq(int i, int j) const;
    tMatrixXd GetTotalDofddJKw_dqq(int i, int j) const;
    tMatrixXd GetTotalDofddJKx_dqq(int i, int j, char type) const;
protected:
    tVector3d link_vel, link_omega; // the lin vel and ang vel w.r.t the COM of
                                    // this link in world frame
    int col_group;
};

#endif // ROBOT_LINK_H
