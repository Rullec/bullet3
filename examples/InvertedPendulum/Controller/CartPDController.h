#pragma once
#include "Controller.h"

class cCartPDController : public cInvertPendulumController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cCartPDController(cRobotModelDynamics *model);
    virtual void UpdateCtrl(double dt) override final;
};