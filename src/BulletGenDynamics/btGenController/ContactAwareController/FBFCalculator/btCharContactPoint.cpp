#include "btCharContactPoint.h"
#include "BulletGenDynamics/btGenModel/RobotModelDynamics.h"
btCharContactPt::btCharContactPt(int c_id) : btGenContactPairData(c_id)
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
    btGenContactPairData::Init(dt, manifold, contact_id_in_manifold);

    if (mIsMBSelfCollision == true)
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
    mContactPosOnA = mWorldPos;
    mContactPosOnB = mWorldPos;
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
        mWorldPos = mContactPosOnA;
    }

    else if (eColObjType::RobotCollder == mBodyB->GetType())
    {
        mCollider = dynamic_cast<btGenRobotCollider *>(mBodyB);
        mWorldPos = mContactPosOnB;
    }

    mWorldPos[3] = 1;
    auto link = mCollider->mModel->GetLinkById(mCollider->mLinkId);
    mLocalPos =
        btMathUtil::InverseTransform(link->GetGlobalTransform()) * mWorldPos;

    tVector new_world_pos = link->GetGlobalTransform() * mLocalPos;
    tVector diff = new_world_pos - mWorldPos;
    if (diff.norm() > 1e-6)
    {
        std::cout << "[error] convert to local pos error\n";
        std::cout << "local pos = " << mLocalPos.transpose() << std::endl;
        std::cout << "world pos = " << mWorldPos.transpose() << std::endl;
        std::cout << "restored world pos = " << new_world_pos.transpose()
                  << std::endl;
        exit(1);
    }
    mCollider->mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(
        mCollider->mLinkId, mWorldPos.segment(0, 3), mJac);
    // std::cout << "world pos = " << mWorldPos.transpose() << std::endl;
    // std::cout << "local pos = " << mLocalPos.transpose() << std::endl;
}