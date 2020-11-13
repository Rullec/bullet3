#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class cRobotModelDynamics;
class btGeneralizeWorld;
class btGenPDController : public btGenControllerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenPDController(btGeneralizeWorld *world);
    virtual ~btGenPDController();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &config) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;
    void SetPDTargetq(const tVectorXd &q);
    void SetPDTargetqdot(const tVectorXd &qdot);
    void ApplyGeneralizedTau(double timestep);

protected:
    cRobotModelDynamics *mModel;
    double mTorqueLim; // max torque limit for stability
    double mKpConstant, mKdConstant;
    tVectorXd mKp, mKd; // PD coefs
    tVectorXd mTargetq, mTargetqdot;
    bool mEnableSPD; // enable stable pd control
};