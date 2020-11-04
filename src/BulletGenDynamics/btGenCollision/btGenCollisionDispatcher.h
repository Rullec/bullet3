#pragma once
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include <string>
namespace Json
{
class Value;
};
class cRobotModelDynamics;
class btTraj;
class btGeneralizeWorld;
class btGenContactAwareController;
class btGenCollisionDispatcher : public btCollisionDispatcher
{
public:
    btGenCollisionDispatcher(const Json::Value &conf,
                             btGeneralizeWorld *world,
                             btCollisionConfiguration *collisionConfiguration);
    void SetController(btGenContactAwareController *ctrl);
    void SetModel(cRobotModelDynamics *model);
    virtual ~btGenCollisionDispatcher();
    void Update();

protected:
    void ParseConf(const Json::Value &conf);
    std::string mCollisionType;
    cRobotModelDynamics *mModel;
    btGenContactAwareController *mController;

    btGeneralizeWorld *mWorld;

    void ClearModelRelatedContact();
    void RestoreContactInfoFromTraj();
};