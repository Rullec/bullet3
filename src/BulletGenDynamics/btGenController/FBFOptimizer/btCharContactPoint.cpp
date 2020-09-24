#include "btCharContactPoint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
btCharContactPt::btCharContactPt(int c_id) : btGenContactPointData(c_id)
{
    mCollider = nullptr;
    mLocalPos = tVector::Zero();
    mWorldPos = tVector::Zero();
    mJac.resize(0, 0);
    mStatus = eContactStatus::INVALID_CONTACT_STATUS;
}
void btCharContactPt::Init(double dt, btPersistentManifold *manifold,
                           int contact_id_in_manifold)
{
    btGenContactPointData::Init(dt, manifold, contact_id_in_manifold);

    if (mIsSelfCollision == true)
    {
        std::cout << "[error] CalcContactStatus cannot handle self "
                     "collsion at this moment\n";
        // exit(0);
    }
}

void btCharContactPt::Init(const tVector &world_pos,
                           btGenRobotCollider *collider)
{
    mWorldPos = world_pos;
    mCollider = collider;
    mBodyA = mCollider;
    mBodyB = mCollider;
    mContactPtOnA = mWorldPos;
    mContactPtOnB = mWorldPos;
}
bool btCharContactPt::IsMultibodyInvolved(cRobotModelDynamics *model)
{
    bool involved = false;
    if (eColObjType::RobotCollder == mBodyA->GetType())
    {
        involved |= dynamic_cast<btGenRobotCollider *>(mBodyA)->mModel == model;
    }
    else if (eColObjType::RobotCollder == mBodyB->GetType())
    {
        involved |= dynamic_cast<btGenRobotCollider *>(mBodyB)->mModel == model;
    }
    return involved;
}
void btCharContactPt::CalcCharacterInfo()
{
    if (eColObjType::RobotCollder == mBodyA->GetType())
    {
        mCollider = dynamic_cast<btGenRobotCollider *>(mBodyA);
        mWorldPos = mContactPtOnA;
    }

    else if (eColObjType::RobotCollder == mBodyB->GetType())
    {
        mCollider = dynamic_cast<btGenRobotCollider *>(mBodyB);
        mWorldPos = mContactPtOnB;
    }

    mWorldPos[3] = 1;
    auto link = mCollider->mModel->GetLinkById(mCollider->mLinkId);
    mLocalPos =
        btMathUtil::InverseTransform(link->GetGlobalTransform()) * mWorldPos;

    mCollider->mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
        mCollider->mLinkId, mWorldPos.segment(0, 3), mJac);
    // std::cout << "world pos = " << mWorldPos.transpose() << std::endl;
    // std::cout << "local pos = " << mLocalPos.transpose() << std::endl;
}