#include "Simbicon3dController.h"
#include "BulletGenDynamics/btGenModel/Joint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
btGenSimbicon3dController::btGenSimbicon3dController(btGeneralizeWorld *world)
    : btGenSimbiconControllerBase(world,
                                  ebtGenControllerType::Simbicon3dController)
{
}
btGenSimbicon3dController::~btGenSimbicon3dController() {}
void btGenSimbicon3dController::Init(cRobotModelDynamics *model,
                                     const std::string &conf)
{
    btGenSimbiconControllerBase::Init(model, conf);

    // confirm this model is in 3d but not bipedal
    Joint *root = dynamic_cast<Joint *>(
        mModel->GetJointById(GetJointByPartialName("root")));
    Joint *left_hip = dynamic_cast<Joint *>(
        mModel->GetJointById(GetJointByPartialName("LeftLeg")));
    Joint *right_hip = dynamic_cast<Joint *>(
        mModel->GetJointById(GetJointByPartialName("RightLeg")));

    BTGEN_ASSERT(root->GetJointType() == JointType::NONE_JOINT);
    BTGEN_ASSERT(left_hip->GetJointType() == JointType::SPHERICAL_JOINT);
    BTGEN_ASSERT(right_hip->GetJointType() == JointType::SPHERICAL_JOINT);
}
void btGenSimbicon3dController::Update(double dt)
{
    btGenSimbiconControllerBase::Update(dt);
}
void btGenSimbicon3dController::Reset() {}

/**
 * \brief           Build the balance control parameter
*/
void btGenSimbicon3dController::BuildBalanceCtrl(const Json::Value &conf)
{
    mCd_forward = btJsonUtil::ParseAsDouble("Cd_forward", conf);
    mCv_forward = btJsonUtil::ParseAsDouble("Cv_forward", conf);
    mCd_tanget = btJsonUtil::ParseAsDouble("Cd_tanget", conf);
    mCv_tanget = btJsonUtil::ParseAsDouble("Cv_tanget", conf);
}
void btGenSimbicon3dController::BalanceUpdateTargetPose(
    tVectorXd &target_pose) const
{
    BTGEN_ASSERT(false);
}