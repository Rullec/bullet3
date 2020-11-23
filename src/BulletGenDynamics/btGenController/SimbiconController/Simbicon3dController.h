#pragma once
#include "SimbiconControllerBase.h"

/**
 * \brief           Simbicon controller implemention for 3d character (not bipedal)
*/
class btGenSimbicon3dController : public btGenSimbiconControllerBase
{
public:
    btGenSimbicon3dController(btGeneralizeWorld *world);
    virtual ~btGenSimbicon3dController();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    double mCd_forward,
        mCv_forward; // balance control param along with the walking direction
    double mCd_tanget,
        mCv_tanget; // balance control param vertical to the walking direction, literally left & right

    virtual void BuildBalanceCtrl(const Json::Value &conf);
    virtual void UpdatePDController(const tVectorXd &tar_pose);
    virtual void BalanceUpdateTargetPose(tVectorXd &target_pose) const;
};