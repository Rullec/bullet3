#include "btGenCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletGenDynamics/btGenController/ContactAwareController/btGenContactAwareController.h"
#include "BulletGenDynamics/btGenController/Trajectory/btTraj.h"
#include "BulletGenDynamics/btGenController/Trajectory/btTrajContactMigrator.h"
#include "BulletGenDynamics/btGenModel/RobotCollider.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenUtil/JsonUtil.h"
#include "BulletGenDynamics/btGenWorld.h"
#include <iostream>
#include <map>

btGenCollisionDispatcher::btGenCollisionDispatcher(
    const Json::Value &conf, btGeneralizeWorld *world,
    btCollisionConfiguration *col)
    : btCollisionDispatcher(col), mWorld(world)
{
    mCollisionType = "";
    mController = nullptr;
    mModel = nullptr;
    std::cout << "[log] gen collision dispatcher is init\n";
    ParseConf(conf);
}

void btGenCollisionDispatcher::SetController(btGenControllerBase *ctrl)
{
    mController = ctrl;
}
btGenCollisionDispatcher::~btGenCollisionDispatcher() {}

void btGenCollisionDispatcher::SetModel(cRobotModelDynamics *model)
{
    mModel = model;
}

/**
 * \brief           Update the dispatcher after each type of collision detection
*/
void btGenCollisionDispatcher::Update()
{
    map_colid_manifold.clear();
    // std::cout << "collision dispatcher update in mode: " << mCollisionType
    //           << std::endl;
    if (mCollisionType == "Native")
    {
    }
    else if (mCollisionType == "FromTheTraj")
    {
        std::cout << "[warn] collision dispatcher works in " << mCollisionType
                  << " mode\n";
        auto contact_aware_controller =
            dynamic_cast<btGenContactAwareController *>(mController);
        if (contact_aware_controller == nullptr)
        {
            std::cout << "[error] btGenCollisionDispatcher: contact aware ctrl "
                         "cannot "
                         "be null when the mode = "
                      << mCollisionType << std::endl;
            exit(0);
        }
        ClearModelRelatedContact();
        RestoreContactInfoFromTraj(contact_aware_controller);
    }
    else if (mCollisionType == "OnlySupposedPoint")
    {
        std::cout << "[warn] collision dispatcher works in " << mCollisionType
                  << " mode\n";
        auto contact_aware_controller =
            dynamic_cast<btGenContactAwareController *>(mController);
        if (contact_aware_controller == nullptr)
        {
            std::cout << "[error] btGenCollisionDispatcher: contact aware ctrl "
                         "cannot "
                         "be null when the mode = "
                      << mCollisionType << std::endl;
            exit(0);
        }
        ClearModelRelatedContact();
        AddContactInfoFromSupposedInfo(contact_aware_controller);
    }
    else
    {
        std::cout << "[error] unrecognize dispatcher type " << mCollisionType
                  << std::endl;
        exit(1);
    }
}

void btGenCollisionDispatcher::ParseConf(const Json::Value &conf)
{
    mCollisionType =
        btJsonUtil::ParseAsString("collision_detection_mode", conf);
}

/**
 * \brief       delete all contact points info related to the robot model
*/
void btGenCollisionDispatcher::ClearModelRelatedContact()
{
    if (mModel == nullptr)
        std::cout << "[error] GenCollisionDispatcher: model is empty\n",
            exit(1);

    // remove the pairs involved with multibody
    auto broad_phase = mWorld->GetBroadphase();
    {
        btBroadphasePairArray &array =
            broad_phase->getOverlappingPairCache()->getOverlappingPairArray();
        for (int i = 0; i < array.size(); i++)
        {
            auto col_obj0 = static_cast<btCollisionObject *>(
                array[i].m_pProxy0->m_clientObject);
            auto gen_obj0 = dynamic_cast<btGenRobotCollider *>(col_obj0);
            auto col_obj1 = static_cast<btCollisionObject *>(
                array[i].m_pProxy1->m_clientObject);
            auto gen_obj1 = dynamic_cast<btGenRobotCollider *>(col_obj1);
            if (gen_obj0 != nullptr || gen_obj1 != nullptr)
            {
                broad_phase->getOverlappingPairCache()->cleanOverlappingPair(
                    array[i], this);
            }
        }
    }

    // remove manifolds
    int raw_num = m_manifoldsPtr.size();
    for (int i = 0; i < m_manifoldsPtr.size(); i++)
    {
        auto mani = m_manifoldsPtr[i];
        auto body0 = dynamic_cast<const btGenRobotCollider *>(mani->getBody0());
        auto body1 = dynamic_cast<const btGenRobotCollider *>(mani->getBody1());
        if ((body0 != nullptr && mModel == body0->mModel) ||
            (body1 != nullptr && mModel == body1->mModel))
        {
            this->releaseManifold(mani);
            i--;
            for (int j = 0; j < m_manifoldsPtr.size(); j++)
            {
                m_manifoldsPtr[j]->m_index1a = j;
            }
        }
    }
    int after_num = m_manifoldsPtr.size();

    printf("[log] GenDispatcher:: clear model related contact mani from %d to "
           "%d\n",
           raw_num, after_num);
}

/**
 * \brief       restore contact info from the ref traj
*/
void btGenCollisionDispatcher::RestoreContactInfoFromTraj(
    btGenContactAwareController *contact_aware_ctrl)
{
    auto mRefTraj = contact_aware_ctrl->GetRefTraj();
    int frameid = contact_aware_ctrl->GetRefFrameId();
    // btTraj * mRefTraj =
    const auto &contact_force_array = mRefTraj->mContactForce[frameid];

    // iterative on all contact points. if this pair has been added, add the contact pt; if not, create new manifold and add.
    // begin to create new contact points and new manifold

    const btCollisionObject *ground = mWorld->GetGround();
    int ground_col_id = ground->getWorldArrayIndex();
    for (int i = 0; i < contact_force_array.size(); i++)
    {
        // 1. get current contact force
        auto &cur_force = contact_force_array[i];
        if (cur_force->mIsSelfCollision == true)
            continue;

        // 2. get link id
        btGenRobotCollider *link_collider =
            mModel->GetLinkCollider(cur_force->mLinkId);
        int link_col_id = link_collider->getWorldArrayIndex();

        // 3. find the manifold
        btPersistentManifold *mani =
            FindOrCreateManifold(link_collider, ground);

        // 3.2 put this contact point into the manifold
        tVector contact_pos_world =
            mModel->GetLinkById(link_collider->mLinkId)->GetGlobalTransform() *
            cur_force->mLocalPos;
        AddContactPointInManifold(mani, link_col_id, ground_col_id,
                                  cur_force->mLocalPos, contact_pos_world,
                                  tVector(0, 1, 0, 0), -1e-4);
        std::cout << "[log] GenDispatcher: put contact point "
                  << contact_pos_world.transpose() << " for body "
                  << link_col_id << " and " << ground_col_id << std::endl;
    }
}

/**
 * \brief               Fetch the supposed contact info(only 2 points at each foot) from the contact-aware controller, 
 * do contact detection by our self (w.r.t the ground)
 * if collided, add them in the manifold
 * else continue
*/
void btGenCollisionDispatcher::AddContactInfoFromSupposedInfo(
    btGenContactAwareController *contact_aware_ctrl)
{
    // 1. get the supposed info, which means that, we should check the file and reload it
    std::string supposed_contact_path =
        contact_aware_ctrl->GetSupposedContactInfo();
    btTrajContactMigrator::tGivenContactPtInfo info =
        btTrajContactMigrator::LoadGivenContactPts(mModel,
                                                   supposed_contact_path);
    btTrajContactMigrator::tGivenContactPtInfo::iterator it = info.begin();
    const btCollisionObject *ground = this->mWorld->GetGround();
    double ground_height = 0;
    while (it != info.end())
    {
        int link_id = it->first;
        const tEigenArr<btTrajContactMigrator::tGivenContactPt> &pt_array =
            it->second;
        auto link = mModel->GetLinkById(link_id);
        auto link_collider = mModel->GetLinkCollider(link_id);
        for (auto &pt : pt_array)
        {
            tVector global_pos = link->GetGlobalTransform() * pt.mLocalPos;

            // we confirm collision here
            if (global_pos[1] < (ground_height + 5e-3))
            {
                // collided! add the contact point
                std::cout << "[log] btGenCollisionDispatcher: add contact "
                             "point from supposed info, link id "
                          << link_id << " pos " << global_pos.transpose()
                          << std::endl;
                auto mani = FindOrCreateManifold(link_collider, ground);
                AddContactPointInManifold(
                    mani, link_collider->getWorldArrayIndex(),
                    ground->getWorldArrayIndex(), pt.mLocalPos, global_pos,
                    tVector(0, 1, 0, 0), -1e-4);
            }
        }
        it++;
    }
}

/**
 * \brief               Given the contact pair id (body0 and body1 id), find the manifold. if it doesn't exist, create a new one and return
 * it's an elementart tools
*/
btPersistentManifold *btGenCollisionDispatcher::FindOrCreateManifold(
    const btGenRobotCollider *link_collider, const btCollisionObject *ground)
{
    int big_id = link_collider->getWorldArrayIndex(),
        small_id = ground->getWorldArrayIndex();
    if (big_id < small_id)
        btMathUtil::Swap(big_id, small_id);
    std::pair<int, int> colid_pair(small_id, big_id);

    // 3.1 if we hasn't found it, create and insert
    std::map<std::pair<int, int>, btPersistentManifold *>::iterator it =
        map_colid_manifold.find(colid_pair);
    if (it == map_colid_manifold.end())
    {
        printf("[log] failed to find the manifold for body id pair %d %d, "
               "create a new one\n",
               small_id, big_id);

        // link_collider is always the link collider (suppose no self collision)

        map_colid_manifold[colid_pair] = getNewManifold(link_collider, ground);
        std::cout << "create new manifold idxa = "
                  << map_colid_manifold[colid_pair]->m_index1a << std::endl;
        it = map_colid_manifold.find(colid_pair);
    }
    return it->second;
}

/**
 * \brief           Add a contact point into the manifold
*/
void btGenCollisionDispatcher::AddContactPointInManifold(
    btPersistentManifold *mani, int link_col_id, int ground_col_id,
    const tVector &pos_local, const tVector &pos_global, const tVector &normal,
    double distance)
{
    assert(distance < 0); // penetration distance, should be negative
    btManifoldPoint pt(

        btBulletUtil::tVectorTobtVector(pos_local),
        btBulletUtil::tVectorTobtVector(pos_local),
        btBulletUtil::tVectorTobtVector(normal),
        distance); // pointA, pointB, normalToBody0, distance <0
    pt.m_partId0 = link_col_id;
    pt.m_partId1 = ground_col_id;
    pt.m_positionWorldOnA = btBulletUtil::tVectorTobtVector(pos_global);
    pt.m_positionWorldOnB = btBulletUtil::tVectorTobtVector(pos_global);

    mani->addManifoldPoint(pt);
}