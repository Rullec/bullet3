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
    tMatrix &GetMWQQ(int i, int j) override;

    //===============For RigidBody Dynamics===============
    void ComputeMassMatrix() override;
    void ComputedMdq(tEigenArr<tMatrixXd> &dMdq);
    void ComputeJKv_dot(tVectorXd &q_dot, tVector3d &p) override;
    void ComputeJKw_dot(tVectorXd &q_dot) override;
    void ComputeDJkvdq(tVector3d &p) override;
    void ComputeDJkwdq() override;
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

protected:
    tVector3d link_vel, link_omega; // the lin vel and ang vel w.r.t the COM of
                                    // this link in world frame
    int col_group;
};

#endif // ROBOT_LINK_H
