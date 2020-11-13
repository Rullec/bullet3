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
    void ComputedMassMatrixdq_global_freedom(tEigenArr<tMatrixXd> &dMdq);
    void ComputedMassMatrixdq_total_freedom(tEigenArr<tMatrixXd> &dMdq);
    tMatrixXd ComputeCoriolisMatrixReduced(const tVectorXd &qdot);
    void ComputedCoriolisMatrixdqReduced(const tVectorXd &qdot,
                                         tEigenArr<tMatrixXd> &dCdq);
    void ComputedCoriolisMatrixdqdotReduced(tEigenArr<tMatrixXd> &dCdqdot);

    void ComputeCoriolisMatrixReduced_part1(const tVectorXd &qdot,
                                            tMatrixXd &C_part1);
    void ComputeCoriolisMatrixReduced_part2(const tVectorXd &qdot,
                                            tMatrixXd &C_part2);

    void ComputeJKv_dot(const tVectorXd &q_dot, const tVector3d &p) override;
    void ComputeJKw_dot(const tVectorXd &q_dot) override;
    void ComputeDJkvdq(const tVector3d &p) override;
    void ComputeDJkwdq() override;
    void ComputeDDJkvddq(const tVector3d &p) override;
    void ComputeDDJkwdqq() override;
    void ComputedJkvdot_dq(const tVectorXd &qdot, const tVector3d &p) override;
    void ComputedJkwdot_dq(const tVectorXd &qdot) override;
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
    // tVector3d GetdddJkw_dqqq(int i, int j, int k) const;
    tMatrixXd GetTotalDofddJKv_dqq(int i, int j) const;
    tMatrixXd GetTotalDofddJKw_dqq(int i, int j) const;
    tMatrixXd GetTotalDofddJKx_dqq(int i, int j, char type) const;
    tMatrixXd GetdJkvdotdq(int dof);
    tMatrixXd GetdJkwdotdq(int dof);
    tMatrixXd GetdJkdotdq(int dof);
    tMatrixXd GetdJkdotdqdot(int dof);

    EIGEN_V_MATXD GetddJKw_dqq_last_channel(int channel_id);

    // ================ forbidden methods begin ===============
    virtual void SetFreedomValueDot(int id, double v) override final;
    virtual void GetFreedomValueDot(int id, double &v) override final;
    virtual void SetFreedomValueDot(std::vector<double> &v) override final;
    virtual void GetFreedomValueDot(std::vector<double> &v) override final;
    // ================ forbidden methods end ===============
protected:
    tVector3d link_vel, link_omega; // the lin vel and ang vel w.r.t the COM of
                                    // this link in world frame
    int col_group;
    void
    ComputedCoriolisMatrixdqReduced_part1(const tVectorXd &qdot,
                                          tEigenArr<tMatrixXd> &dCdq_part1);
    void ComputedCoriolisMatrixdq_part11(const tVectorXd &qdot,
                                         tEigenArr<tMatrixXd> &dCdq_part11);
    void
    ComputedCoriolisMatrixdqReduced_part2(const tVectorXd &qdot,
                                          tEigenArr<tMatrixXd> &dCdq_part1);
    void ComputeCoriolisMatrix_part11(const tVectorXd &qdot,
                                      tMatrixXd &C_part11);
    void ComputeCoriolisMatrix_part12(const tVectorXd &qdot,
                                      tMatrixXd &C_part12);
};

#endif // ROBOT_LINK_H
