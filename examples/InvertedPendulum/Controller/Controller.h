#pragma once
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
class cInvertPendulumController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cInvertPendulumController(cRobotModelDynamics *model);
    virtual void UpdateCtrl(double dt) = 0;

protected:
    cRobotModelDynamics *mModel;
};