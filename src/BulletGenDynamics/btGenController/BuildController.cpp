#include "BulletGenDynamics/btGenController/BuildController.h"
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/PDController/btGenSimbiconSPDController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/Simbicon3dController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconBipedalController.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
btGenControllerBase *BuildController(btGeneralizeWorld *world,
                                     const std::string &path)
{
    Json::Value root;
    btJsonUtil::LoadJson(path, root);

    std::string type = btJsonUtil::ParseAsString("ctrl_type", root);

    btGenControllerBase *ctrl = nullptr;
    for (int i = 0; i < ebtGenControllerType::BTGEN_NUM_CONTROLLER_TYPE; i++)
    {
        if (type == gbtGenControllerTypeStr[i])
        {
            switch (static_cast<ebtGenControllerType>(i))
            {
            case ebtGenControllerType::PDController:
                ctrl = new btGenPDController(world);
                break;
            case ebtGenControllerType::SimbiconSPDController:
                ctrl = new btGenSimbiconSPDController(world);
                break;
            case ebtGenControllerType::ContactAwareController:
                ctrl = new btGenContactAwareController(world);
                break;
            case ebtGenControllerType::SimbiconBipedController:
                ctrl = new btGenSimbiconBipedalController(world);
                break;
            case ebtGenControllerType::Simbicon3dController:
                ctrl = new btGenSimbicon3dController(world);
                break;
            default:
                break;
            }
        }
    }
    if (ctrl == nullptr)
    {
        printf("[error] BuildController type %s failed\n", type.c_str());
        exit(1);
    }
    return ctrl;
}
