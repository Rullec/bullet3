#pragma once
#include "BulletGenDynamics/btGenController/btGenTargetCalculator.h"

class btGenNQRCalculator : public btGenTargetCalculator
{
public:
    btGenNQRCalculator();
    ~btGenNQRCalculator();
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
};