#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconController.h"

btGenSimbiconController::btGenSimbiconController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::SimbiconController, world)
{
}
void btGenSimbiconController::Init(cRobotModelDynamics *model,
                                   const std::string &conf)
{
    btGenControllerBase::Init(model, conf);
    // 1. check the skeleton(fixed now)
    // 2. build FSM
}

/**
 * \brief               Update simbicon controller, calculate & apply the control force
 * 1. input current time, input contact points, update FSM, output target pose
 * 2. input target pose into balance feedback module, output target pose
 * 3. input the target pose into PD controller, output control force
 *       3.1 for stance hip, use the inverse torque and don't track the ref pose
 *       3.2 for torso and swing hip, the torque is calculated in world frame
 *       3.3 for other joints, just track the local angle
*/
void btGenSimbiconController::Update(double dt)
{
    btGenControllerBase::Update(dt);
}
void btGenSimbiconController::Reset() {}