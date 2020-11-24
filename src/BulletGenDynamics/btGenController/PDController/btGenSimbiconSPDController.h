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
    void SetJointId(int swing_hip, int stance_hip, int root_id);
    virtual void Reset() override;

protected:
    bool mJointIdUpdated; // whether the Joint id is updated at this moment
    bool mTargetUpdated;  // target pose updated or not
    int mSwingHipId, mStanceHipId, mRootId;
    bool EnableSimbiconSPD() const;
    void CalculateControlForcesSimbiconSPD(double dt,
                                           tEigenArr<btGenPDForce> &pd_forces);
    void VerifySimbiconSPD(double dt, tEigenArr<btGenPDForce> &pd_forces);
};