#include "CartPDController.h"
#include <iostream>

cCartPDController::cCartPDController(cRobotModelDynamics *model)
    : cInvertPendulumController(model)
{
}

void cCartPDController::UpdateCtrl(double dt)
{
    int dof = mModel->GetNumOfFreedom();
    double theta = mModel->Getq()[dof - 1],
           theta_dot = mModel->Getqdot()[dof - 1];
    double tar_theta = 0, tar_theta_dot = 0;
    double theta_err = tar_theta - theta,
           theta_dot_err = tar_theta_dot - theta_dot;
    double Kp = 200, Kd = 50;
    double f = Kp * theta_err + Kd * theta_dot_err;
    double force_lim = 1000;
    if (std::fabs(f) > force_lim)
        f *= 1.0 / std::fabs(f) * force_lim;
    std::cout << "[pd] apply " << f << std::endl;
    mModel->ApplyForce(0, tVector(0, 0, f, 0),
                       btMathUtil::Expand(mModel->GetRoot()->GetWorldPos(), 1));
}