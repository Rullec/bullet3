#pragma once
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"

class Joint;
class btGenShowLimitController : public btGenControllerBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    btGenShowLimitController(btGeneralizeWorld *world);
    virtual ~btGenShowLimitController();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    int mCurFrame;
    // Joint *mTargetJoint;
    bool mDrawLimit;
    void DrawLimit(Joint *joint);
    tVector GetJointOutgoingDir(Joint *joint) const;
    tVectorXd mInitPose;
};