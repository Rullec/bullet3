#include "BulletGenDynamics/btGenController/btGenTargetCalculator.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
#include "BulletGenDynamics/btGenWorld.h"
btGenTargetCalculator::btGenTargetCalculator()
{
    mModel = nullptr;
    mTraj = nullptr;
    mWorld = nullptr;
}

btGenTargetCalculator::~btGenTargetCalculator() {}

/**
 * \brief           Set the traj to the controller
*/
void btGenTargetCalculator::SetTraj(btTraj *traj_) { mTraj = traj_; }

void btGenTargetCalculator::Init(btGeneralizeWorld *world,
                                 const Json::Value &conf)
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