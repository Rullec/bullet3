#include "BulletGenDynamics/btGenController/btGenTargetCalculator.h"
#include "../examples/CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenWorld.h"

btGenTargetCalculator::btGenTargetCalculator()
{
    mModel = nullptr;
    mTraj = nullptr;
    mWorld = nullptr;
    mDrawPointsList.clear();
    mRefFrameId = -1;
}

btGenTargetCalculator::~btGenTargetCalculator() { ClearDrawPoints(); }

/**
 * \brief           Set the traj to the controller
*/
void btGenTargetCalculator::SetTraj(btTraj *traj_) { mTraj = traj_; }

void btGenTargetCalculator::Init(btGeneralizeWorld *world,
                                 const std::string conf)
{
    mWorld = world;
    mModel = mWorld->GetMultibody();
    num_of_freedom = mModel->GetNumOfFreedom();
    num_of_underactuated_freedom = num_of_freedom - 6;
}
/**
 * 
 * \brief           Given the bullet GUI pointer, we can draw some custom stuff in this class
*/
void btGenTargetCalculator::SetBulletGUIHelperInterface(
    struct GUIHelperInterface *inter)
{
    mBulletGUIHelper = inter;
}

/**
 * \brief           do some check before calc the target
*/
void btGenTargetCalculator::PreCalcTarget(double dt, int target_id)
{
    if (mTraj == nullptr)
    {
        std::cout
            << "[error] the traj hasn't been set in the FBFCalculator, exit";
        exit(0);
    }
    mRefFrameId = target_id;
    mdt = dt;
}

// struct GUIHelperInterface *gGUIHelper;
void btGenTargetCalculator::DrawPoint(const tVector3d &pos, double radius)
{
    if (mBulletGUIHelper == nullptr)
        return;
    btCollisionShape *colShape = nullptr;
    btCollisionObject *obj = new btCollisionObject();
    colShape = new btSphereShape(btScalar(radius));
    btTransform trans;
    // trans.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    trans.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    obj->setWorldTransform(trans);
    obj->setCollisionShape(colShape);
    obj->setCollisionFlags(0);
    mWorld->GetInternalWorld()->addCollisionObject(obj, 0, 0);
    mDrawPointsList.push_back(obj);

    auto inter_world = mWorld->GetInternalWorld();
    // std::cout << "draw point " << pmCollisionObjects() << std::endl;
}

void btGenTargetCalculator::ClearDrawPoints()
{
    if (mBulletGUIHelper == nullptr)
        return;
    auto inter_world = mWorld->GetInternalWorld();
    // std::cout << "clear points num = " << mDrawPointsList.size()
    //           << " now = " << inter_world->getCollisionObjectArray().size()
    //           << std::endl;

    for (auto &pt : mDrawPointsList)
    {
        // inter_world->getCollisionObjectArray().remove(pt);
        delete pt->getCollisionShape();
        mWorld->GetInternalWorld()->removeCollisionObject(pt);
        mBulletGUIHelper->removeGraphicsInstance(pt->getUserIndex());
        delete pt;
    }
    // std::cout << "[debug] clear points " << mDrawPointsList.size() << std::endl;
    mDrawPointsList.clear();
}
