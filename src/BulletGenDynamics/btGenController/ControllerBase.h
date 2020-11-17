#ifndef CONTROLLER_BASW_H_
#define CONTROLLER_BASW_H_
#include <string>
enum ebtGenControllerType
{
    PDController,
    ContactAwareController,
    SimbiconController,
    BTGEN_NUM_CONTROLLER_TYPE
};

static const std::string
    gbtGenControllerTypeStr[ebtGenControllerType::BTGEN_NUM_CONTROLLER_TYPE] = {
        "PDController",
        "ContactAwareController",
        "SimbiconController",
};
class btGeneralizeWorld;
class cRobotModelDynamics;
class btGenControllerBase
{
public:
    btGenControllerBase(ebtGenControllerType type, btGeneralizeWorld *world);
    virtual void Init(cRobotModelDynamics *model, const std::string &conf);
    virtual void Update(double dt);
    virtual void Reset() = 0;
    ebtGenControllerType GetCtrlType() const;

protected:
    double mTime;
    cRobotModelDynamics *mModel;
    btGeneralizeWorld *mWorld;
    ebtGenControllerType mCtrlType;
    double mCurdt;
};
#endif /*CONTROLLER_BASW_H_*/