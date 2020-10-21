#include "Controller.h"
#include <iostream>
cInvertPendulumController::cInvertPendulumController(cRobotModelDynamics *model)
{
    mModel = model;

    if (mModel == nullptr || mModel->GetNumOfJoint() != 2)
    {
        std::cout << "[error] Invert Pendulum Controller: invalid model\n";
        exit(1);
    }
}