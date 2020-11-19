#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconControllerBase.h"

/**
 * \brief           Bipedal simbicon controller
*/
class btGenSimbiconBipedalController : public btGenSimbiconControllerBase
{
public:
    btGenSimbiconBipedalController(btGeneralizeWorld *world);
    virtual ~btGenSimbiconBipedalController();
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    double mCd, mCv; // balance control param

    virtual void BuildBalanceCtrl(const Json::Value &conf);
    virtual void BalanceUpdateTargetPose(tVectorXd &target_pose) const;
};