#pragma once
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenTargetCalculator.h"

/**
 * Nonlinear quadratic regular implemention based on 2009 TOG "Contact-aware Nonlinear Control of Dynamic Characters"
 * 1. SetTraj: given the mocap data, the nqr calculator will do precompution, and put the result in "tNQRFrameInfo"
 * 2. CalcTarget: given the desired frame, calculate the target of control force & character accel.
*/
struct btCharContactPt;
class btGenMBContactForce;
class btGenNQRCalculator : public btGenTargetCalculator
{
public:
    typedef std::vector<btGenMBContactForce *> tContactInfo;
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
        // 3. variable sizes
        int mContactForceSize;      // the size of total contact force
        int mJointControlForceSize; // joint_force_size = total_dof - 6
        int mTotalControlForceSize; // total_control_size = contact_size + joint_force_size

        // 4. total control jacobian and derivations
        tMatrixXd
            mTotalControlJacobian; //  gen_force = mTotalControlJacobian * control_vector[joint, contact]
        tEigenArr<tMatrixXd> m_dControlJac_dq; // d(mTotalControlJacobian)/dq

        // 5. mocap cartesian contact force, mocap control force (n-6), combined get the control vector
        tVectorXd mCartesianContactForce_mocap; // size = 3 * N_contact_number
        tVectorXd mJointControlForce_mocap;     // size = n-6
        tVectorXd
            mTotalControlVector_mocap; // size = 3 * N_contact_number + n-6

        // 6. system linearization result
        /*
            x_{t+1} = G * u + h
            A = dGdx * u + dhdx
            B = G
        */
        tMatrixXd mA; // dGdx * u + dhdx
        tMatrixXd mB; // G

        // 7. mocap state vector [q, qdot]
        tVectorXd mStateVector_mocap;
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

    // std::vector<tSupposedContactPt *> mSupposedContactPt;
    int mStateSize;          // state size = q_size + qdot_size
    double Q_coef;           // state vector x close to origin
    double R_coef;           // control vector u close to origin
    double P_coef;           // control vector u minimize
    std::string mWorkPolicy; // lqr or nqr? policy calculation?

    void CalcNQR();
    // ==================System Equation & Linearalization==================
    tMatrixXd GetG(const tContactInfo &contact_info);
    tVectorXd GetR();
    tVectorXd Geth();
    void GetdGdx(const tContactInfo &contact_info, tEigenArr<tMatrixXd> &dGdx);
    void Getdhdx(tMatrixXd &dhdx);
    void GetdRdq(tMatrixXd &dRdq);
    void GetdRdqdot(tMatrixXd &dRdqdot);
    void VerifydGdx(const tContactInfo &contact_info,
                    const tEigenArr<tMatrixXd> &analytic_dGdx);
    void VerifydRdq(const tMatrixXd &dRdq_ana);
    void VerifydRdqdot(const tMatrixXd &dRdqdot_ana);
    void Verifydhdx(const tMatrixXd &analytic_dhdx);
    tVectorXd Getxnext(const tContactInfo &contact_info, const tVectorXd &x,
                       const tVectorXd &u);
    void CalcSystemLinearzation(const tContactInfo &contact_info,
                                tNQRFrameInfo &info);
    void VerifySystemLinearzation(const tContactInfo &contact_info,
                                  const tMatrixXd &dfdx, const tMatrixXd &dfdu,
                                  const tVectorXd &x, const tVectorXd &u);

    // ===================Handle contact & control forces/jacobian================
    void CalcContactAndControlForce(int frame_id, tNQRFrameInfo &info);
    tMatrixXd CalcTotalControlJacobian(const tContactInfo &contact_info);
    void CalcTotalControl_dJacobiandq(tEigenArr<tMatrixXd> &dJacdq,
                                      const tContactInfo &contact_info);
    void CalcdContactJacobiandq(const tContactInfo &contact_info,
                                tEigenArr<tMatrixXd> &dJacdq);
    void VerifyContactAndControlJacobian(
        const tContactInfo &contact_info, const tNQRFrameInfo &nqr_info,
        const tEigenArr<tMatrixXd> &dJacdq_analytic);

    // ====================Solve ARE, forward the optimal policy==================
    void CalcNQRRiccati(tNQRFrameInfo *cur_info,
                        const tNQRFrameInfo *const next_info);
    void VerifyNQRRiccati();
    tVectorXd CalcControlVector(const tContactInfo &contact_info);
    void ApplyControlVector(const tContactInfo &contact_info,
                            const tNQRFrameInfo &nqr_info,
                            const tVectorXd &control_vector);

    // ==================Tools================
    void CheckFrameId(int frame_id, std::string prefix);
    void CheckModelState(int frame_id, std::string prefix);
    // void InitSupposdedContactPoints(const std::string &root);
    // virtual void PreCalcTarget(double dt, int target_id);
    // void DrawSupposedContactPoints();
};