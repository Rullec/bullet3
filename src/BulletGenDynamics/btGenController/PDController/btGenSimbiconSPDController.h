#pragma once
#include "btGenPDController.h"

/**
 * \brief           A SPD simbicon controller 
*/
class btGenSimbiconSPDController : public btGenPDController
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenSimbiconSPDController(btGeneralizeWorld *world);
    virtual ~btGenSimbiconSPDController();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &config) override;
    virtual void Update(double dt) override;
    virtual void CalculateControlForces(double dt,
                                        tEigenArr<btGenPDForce> &pd_forces);
    virtual void Reset() override;

protected:
};