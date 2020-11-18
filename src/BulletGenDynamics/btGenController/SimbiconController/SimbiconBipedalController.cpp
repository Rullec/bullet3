#include "SimbiconBipedalController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSM.h"
#include "BulletGenDynamics/btGenController/SimbiconController/FSMUtil.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"

btGenSimbiconBipedalController::btGenSimbiconBipedalController(
    btGeneralizeWorld *world)
    : btGenSimbiconControllerBase(world,
                                  ebtGenControllerType::SimbiconBipedController)
{
    mCd = 0;
    mCv = 0;
}

/**
 * \brief           Simbicon build blance control policy (2 coeffs)
*/
void btGenSimbiconBipedalController::BuildBalanceCtrl(const Json::Value &conf)
{
    mCd = btJsonUtil::ParseAsDouble("Cd", conf);
    mCv = btJsonUtil::ParseAsDouble("Cv", conf);
}

/**
 * \brief               Update the target pose by the blance control
 * \param target_pose   target q got from the FSM
 * 1. find the swing hip, 
 * 2. get the COM pos "d" & vel "v" 
 * 3. change the control target of swing hip by theta = theta_d + c_d * d + c_v * v
*/
void btGenSimbiconBipedalController::BalanceUpdateTargetPose(
    tVectorXd &target_pose) const
{
    if (mSwingHip == -1 || mStanceHip == -1)
    {
        printf("[warn] swing/stance hip is -1, balance control disabled\n");
        return;
    }
    auto cur_state = mFSM->GetCurrentState();
    if (mIgnoreBalanceControlInState02 == true &&
        cur_state->GetStateId() != 1 && cur_state->GetStateId() != 3)
    {
        printf("[warn] balance control is ignored in state id %d temporarily\n",
               cur_state->GetStateId());
        return;
    }
    auto swing_hip = dynamic_cast<Joint *>(mModel->GetJointById(mSwingHip));
    BTGEN_ASSERT(swing_hip->GetJointType() == JointType::REVOLUTE_JOINT);

    int offset = swing_hip->GetOffset();
    int size = swing_hip->GetNumOfFreedom();
    BTGEN_ASSERT(size == 1);
    double theta = target_pose[offset];
    tVector3d com_pos = mModel->GetCoMPosition(),
              com_vel = mModel->GetComVelocity();
    double d = com_pos[2] - mModel->GetJointById(GetEndeffector(mStanceHip))
                                ->GetWorldPos()[2],
           v = com_vel[2];

    // minus angle means uplift, for current hips
    target_pose[offset] = -mCd * d - mCv * v + theta;

    printf("[simbicon] d = %.3f, v = %.3f, origin theta %.3f, "
           "result theta %.3f for swing hip %s\n",
           d, v, theta, target_pose[offset], swing_hip->GetName().c_str());
}

void btGenSimbiconBipedalController::Init(cRobotModelDynamics *model,
                                          const std::string &conf)
{
    btGenSimbiconControllerBase::Init(model, conf);
}
void btGenSimbiconBipedalController::Update(double dt)
{
    btGenSimbiconControllerBase::Update(dt);
}
void btGenSimbiconBipedalController::Reset()
{
    btGenSimbiconControllerBase::Reset();
}