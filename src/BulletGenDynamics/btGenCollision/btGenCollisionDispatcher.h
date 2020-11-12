#pragma once
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include <map>
#include <string>
namespace Json
{
class Value;
};
class cRobotModelDynamics;
class btGenRobotCollider;
class btTraj;
class btGeneralizeWorld;
class btGenContactAwareController;
class btGenControllerBase;
class btGenCollisionDispatcher : public btCollisionDispatcher
{
public:
    btGenCollisionDispatcher(const Json::Value &conf, btGeneralizeWorld *world,
                             btCollisionConfiguration *collisionConfiguration);
    void SetController(btGenControllerBase *ctrl);
    void SetModel(cRobotModelDynamics *model);
    virtual ~btGenCollisionDispatcher();
    void Update();

protected:
    void ParseConf(const Json::Value &conf);
    std::string mCollisionType;
    cRobotModelDynamics *mModel;
    btGenControllerBase *mController;

    btGeneralizeWorld *mWorld;

    void ClearModelRelatedContact();
    void RestoreContactInfoFromTraj(btGenContactAwareController * contact_aware_ctrl);
    void AddContactInfoFromSupposedInfo(btGenContactAwareController * contact_aware_ctrl);

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