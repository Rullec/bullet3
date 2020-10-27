#pragma once
#include "BulletGenDynamics/btGenController/btGenTargetCalculator.h"

/**
 * Nonlinear quadratic regular implemention based on 2009 TOG "Contact-aware Nonlinear Control of Dynamic Characters"
 * 1. SetTraj: given the mocap data, the nqr calculator will do precompution, and put the result in "tNQRFrameInfo"
 * 2. CalcTarget: given the desired frame, calculate the target of control force & character accel.
*/
struct btCharContactPt;
class btGenNQRCalculator : public btGenTargetCalculator
{
public:
    btGenNQRCalculator();
    virtual ~btGenNQRCalculator();
    virtual void Init(btGeneralizeWorld *mWorld,
                      const std::string conf) override;
    virtual void SetTraj(btTraj *traj_) override;
    virtual void SetCoef(const Json::Value &conf) override;
    virtual void CalcTarget(double dt, int target_frame_id,
                            tVectorXd &tilde_qddot, tVectorXd &tilde_qdot,
                            tVectorXd &tilde_q, tVectorXd &tilde_tau) override;
    virtual int GetCalculatedNumOfContact() const override;
    virtual void ControlByAdaptionController() override;
    virtual void Reset() override;

protected:
    struct tNQRFrameInfo
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 1. costate equation: \lambda = Sk_mat * \xi + sk_vec
        tMatrixXd Sk_mat;
        tVectorXd sk_vec;
        // 2. coef vector. they should be used as Diagnoal mat
        tVectorXd Q_coef; // state vector x close to origin
        tVectorXd R_coef; // control vector u close to origin
        tVectorXd P_coef; // control vector u minimize
        tNQRFrameInfo();
    };

    tEigenArr<tNQRFrameInfo> mNQRFrameInfos;

    struct tSupposedContactPt
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        tSupposedContactPt(const cRobotModelDynamics *model, int link_id,
                           const tVector &local_pos);
        tVector GetGlobalPos();
        const int mLinkId;
        const tVector mLocalPos;
        const cRobotModelDynamics *mModel;
    };

    std::vector<tSupposedContactPt *> mSupposedContactPt;
    int mStateSize;             // state size = q_size + qdot_size
    int mContactForceSize;      // the size of total contact force
    int mJointControlForceSize; // joint_force_size = total_dof - 6
    int mTotalControlForceSize; // total_control_size = contact_size + joint_force_size
    // methods
    void InitSupposdedContactPoints(const std::string &root);
    void CalcNQR();
    void CalcNQRContactAndControlForce(int frame_id);
    void CalcNQRSystemLinearzation(int frame);
    void CalcNQRRiccati();
    void VerifyNQRSystemLinearzation(int frame);
    void VerifyNQRRiccati();
    // virtual void PreCalcTarget(double dt, int target_id);
    void DrawSupposedContactPoints();
};