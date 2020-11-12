#include "BulletGenDynamics/btGenController/ControllerBase.h"

btGenControllerBase::btGenControllerBase(ebtGenControllerType type,
                                         btGeneralizeWorld *world)
    : mCtrlType(type), mWorld(world)
{
    mModel = nullptr;
    mCurdt = 0;
}

void btGenControllerBase::Init(cRobotModelDynamics *model,
                               const std::string &conf)
{
    mTime = 0;
    mModel = model;
}

ebtGenControllerType btGenControllerBase::GetCtrlType() const
{
    return mCtrlType;
}

void btGenControllerBase::Update(double dt)
{
    mCurdt = dt;
    mTime += dt;
}