#pragma once
#include "Controller.h"
class cLQRController : public cInvertPendulumController
{
public:
    cLQRController(cRobotModelDynamics *model,
                   std::string policy = "zero_point_lagragian");
    virtual void UpdateCtrl(double dt) override final;

protected:
    void GetSystem(tMatrixXd &A, tMatrixXd &B);
    void GetCoef(tMatrixXd &Q, tMatrixXd &R);
    void CtrlBySimpleDyna();
    void GetSystemSimple(tMatrixXd &A, tMatrixXd &B);
    void GetSystemLagragianZeroPoint(tMatrixXd &A, tMatrixXd &B);
    void GetSystemLagragian(tMatrixXd &A, tMatrixXd &B);
    void SolveRicatti(const tMatrixXd &A, const tMatrixXd &B,
                      const tMatrixXd &Q, const tMatrixXd &R, tMatrixXd &P);
    tVectorXd GetState();
    void LQRControl(const tMatrixXd &P);
    const std::string mPolicy;
    tMatrixXd mA, mB;
};