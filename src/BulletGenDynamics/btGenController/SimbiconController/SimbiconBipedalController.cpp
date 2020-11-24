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
    mCd_forward = 0;
    mCv_forward = 0;
    mCd_tangent = 0;
    mCv_tangent = 0;
}

btGenSimbiconBipedalController::~btGenSimbiconBipedalController() {}
/**
 * \brief           Simbicon build blance control policy (2 coeffs)
*/
void btGenSimbiconBipedalController::BuildBalanceCtrl(const Json::Value &conf)
{
    mCd_forward = btJsonUtil::ParseAsDouble("Cd_forward", conf);
    mCv_forward = btJsonUtil::ParseAsDouble("Cv_forward", conf);
    mCd_tangent = btJsonUtil::ParseAsDouble("Cd_tangent", conf);
    mCv_tangent = btJsonUtil::ParseAsDouble("Cv_tangent", conf);
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

    switch (swing_hip->GetJointType())
    {
    case JointType::REVOLUTE_JOINT:
        BalanceUpdateTargetPoseRevoluteHips(target_pose);
        break;
    case JointType::SPHERICAL_JOINT:
        BalanceUpdateTargetPoseSphericalHips(target_pose);
        break;
    default:
        BTGEN_ASSERT(false);
        break;
    };
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

/**
 * \brief           Update the target pose by COM vel and COM pos
*/
void btGenSimbiconBipedalController::BalanceUpdateTargetPoseRevoluteHips(
    tVectorXd &target_pose) const
{
    auto swing_hip = dynamic_cast<Joint *>(mModel->GetJointById(mSwingHip));
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
    target_pose[offset] = -mCd_forward * d - mCv_forward * v + theta;

    printf("[simbicon] d = %.3f, v = %.3f, origin theta %.3f, "
           "result theta %.3f for swing hip %s\n",
           d, v, theta, target_pose[offset], swing_hip->GetName().c_str());
}

void btGenSimbiconBipedalController::BalanceUpdateTargetPoseSphericalHips(
    tVectorXd &target_pose) const
{
    // com position, com velocity, stance foot position in world frame
    tVector3d com_pos = mModel->GetCoMPosition(),
              com_vel = mModel->GetComVelocity();
    tVector3d stance_foot_pos =
        mModel->GetJointById(GetEndeffector(mStanceHip))->GetWorldPos();

    // d and v in world frame
    tVector3d d = com_pos - stance_foot_pos, v = com_vel;

    // do balance control policy in character frame (heading orientation)
    // we assume the upaxis is Y
    double heading = mModel->GetHeading();

    // express the d and v in heading frame
    tVector3d heading_inv_axisangle = tVector3d(0, -heading, 0);
    tMatrix3d head_inv_rotmat =
        btMathUtil::AxisAngleToRotmat(
            btMathUtil::Expand(heading_inv_axisangle, 0))
            .block(0, 0, 3, 3);

    d = head_inv_rotmat * d; // now d is expressed in heading frame
    v = head_inv_rotmat * v; // now v is expressed in heading frame

    auto swing_joint = dynamic_cast<Joint *>(mModel->GetJointById(mSwingHip));

    tVector3d raw_target_euler =
        target_pose.segment(swing_joint->GetOffset(), 3);
    tVector3d target_aa =
        btMathUtil::EulerangleToAxisAngle(
            btMathUtil::Expand(raw_target_euler, 0), btRotationOrder::bt_XYZ)
            .segment(0, 3);
    // 1. update forward target (z axis)
    target_aa[0] += -mCd_forward * d[0] - mCv_forward * v[0];

    // 2. update tangent target (x target)
    target_aa[2] += mCd_tangent * d[2] + mCv_tangent * v[2];

    tVector3d new_target_euler =
        btMathUtil::AxisAngleToEulerAngle(btMathUtil::Expand(target_aa, 0),
                                          btRotationOrder::bt_XYZ)
            .segment(0, 3);
    target_pose.segment(swing_joint->GetOffset(), 3) = new_target_euler;
}