#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconController.h"

btGenSimbiconController::btGenSimbiconController(btGeneralizeWorld *world)
    : btGenControllerBase(ebtGenControllerType::SimbiconController, world)
{
}
void btGenSimbiconController::Init(cRobotModelDynamics *model,
                                   const std::string &conf)
{
    btGenControllerBase::Init(model, conf);
}
void btGenSimbiconController::Update(double dt)
{
    btGenControllerBase::Update(dt);
}
void btGenSimbiconController::Reset() {}