#include "BulletGenDynamics/btGenController/BuildController.h"
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
#include "BulletGenDynamics/btGenController/ControllerBase.h"
#include "BulletGenDynamics/btGenController/PDController/btGenPDController.h"
#include "BulletGenDynamics/btGenController/PDController/btGenSimbiconSPDController.h"
#include "BulletGenDynamics/btGenController/ShowLimitController/ShowLimitController.h"
#include "BulletGenDynamics/btGenController/SimbiconController/SimbiconControllerBase.h"
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
                BTGEN_ASSERT(false);
                // ctrl = new btGenSimbiconSPDController(world);
                break;
            case ebtGenControllerType::ContactAwareController:
                ctrl = new btGenContactAwareController(world);
                break;
            case ebtGenControllerType::SimbiconController:
                ctrl = new btGenSimbiconControllerBase(world);
                break;
            case ebtGenControllerType::ShowLimitController:
                ctrl = new btGenShowLimitController(world);
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
