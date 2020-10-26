#include "BulletGenDynamics/btGenController/NQRCalculator/btGenNQRCalculator.h"

btGenNQRCalculator::btGenNQRCalculator() {}

btGenNQRCalculator::~btGenNQRCalculator() {}

void btGenNQRCalculator::Init(btGeneralizeWorld *mWorld, const std::string conf)
{
}
void btGenNQRCalculator::SetTraj(btTraj *traj_) {}
void btGenNQRCalculator::SetCoef(const Json::Value &conf) {}
void btGenNQRCalculator::CalcTarget(double dt, int target_frame_id,
                                    tVectorXd &tilde_qddot,
                                    tVectorXd &tilde_qdot, tVectorXd &tilde_q,
                                    tVectorXd &tilde_tau)
{
}
int btGenNQRCalculator::GetCalculatedNumOfContact() const { return -1; }
void btGenNQRCalculator::ControlByAdaptionController() {}
void btGenNQRCalculator::Reset() {}