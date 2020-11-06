#pragma once
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include <string>
#include <map>
namespace Json
{
class Value;
};
class cRobotModelDynamics;
class btGenRobotCollider;
class btTraj;
class btGeneralizeWorld;
class btGenContactAwareController;
class btGenCollisionDispatcher : public btCollisionDispatcher
{
public:
    btGenCollisionDispatcher(const Json::Value &conf, btGeneralizeWorld *world,
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
    void AddContactInfoFromSupposedInfo();

    // =============
    btPersistentManifold *
    FindOrCreateManifold(const btGenRobotCollider *link_collider,
                         const btCollisionObject *ground);
    void AddContactPointInManifold(btPersistentManifold *mani, int link_col_id,
                                   int groud_col_id,
                                   const tVector &contact_pos_bt_local,
                                   const tVector &contact_pos_bt_global,
                                   const tVector &normal, double distance);
    std::map<std::pair<int, int>, btPersistentManifold *>
        map_colid_manifold; // map from the collision body id pair to the manifold
};