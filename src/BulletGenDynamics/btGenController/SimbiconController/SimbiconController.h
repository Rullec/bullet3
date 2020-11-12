#include "BulletGenDynamics/btGenController/ControllerBase.h"

/**
 * \brief       2007 SIGGRAPH paper SIMBICON reimplemention, Kangkang Yin. et al
*/
class btGenSimbiconController : public btGenControllerBase
{
public:
    btGenSimbiconController(btGeneralizeWorld *world);
    virtual void Init(cRobotModelDynamics *model,
                      const std::string &conf) override;
    virtual void Update(double dt) override;
    virtual void Reset() override;

protected:
    
};