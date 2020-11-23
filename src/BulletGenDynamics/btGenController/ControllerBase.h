#ifndef CONTROLLER_BASW_H_
#define CONTROLLER_BASW_H_
#include <string>
enum ebtGenControllerType
{
    PDController,
    SimbiconSPDController,
    ContactAwareController,
    SimbiconBipedController,
    Simbicon3dController,
    BTGEN_NUM_CONTROLLER_TYPE
};

static const std::string
    gbtGenControllerTypeStr[ebtGenControllerType::BTGEN_NUM_CONTROLLER_TYPE] = {
        "PDController",           "SimbiconSPDController",
        "ContactAwareController", "SimbiconBipedalController",
        "Simbicon3dController",
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