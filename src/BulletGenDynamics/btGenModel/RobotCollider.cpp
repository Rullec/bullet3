#include "RobotCollider.h"
#include "RobotModelDynamics.h"
#include <iostream>

btGenRobotCollider::btGenRobotCollider(cRobotModelDynamics *model, int link_id,
                                       const std::string &name, int col_group)
    : mModel(model), mLinkId(link_id),
      btGenCollisionObject(eColObjType::RobotCollder, name),
      mColGroup(col_group)
{
    m_checkCollideWith = true;
    m_collisionFlags &= (~btCollisionObject::CF_STATIC_OBJECT);
    m_internalType = CO_COLLISION_OBJECT;
    // std::cout << "[debug] link " << link_id << " col group = " << mColGroup
    // << std::endl;
}

btGenRobotCollider::~btGenRobotCollider()
{
    // if (nullptr != getBroadphaseHandle())
    // {
    // 	std::cout << "begin to release " << getBroadphaseHandle() << std::endl;
    // 	delete getBroadphaseHandle();
    // }
}

btGenRobotCollider *btGenRobotCollider::upcast(btCollisionObject *colObj)
{
    return dynamic_cast<btGenRobotCollider *>(colObj);
}
const btGenRobotCollider *
btGenRobotCollider::upcast(const btCollisionObject *colObj)
{
    return dynamic_cast<const btGenRobotCollider *>(colObj);
}

bool btGenRobotCollider::checkCollideWithOverride(
    const btCollisionObject *co) const
{
    // std::cout << "checkCollideWithOverride called\n";
    // std::cout <<"multibody col flag = " << m_collisionFlags << std::endl;
    const btGenRobotCollider *col_robot = btGenRobotCollider::upcast(co);

    // 1. it doesn't belong to the multibody, collided
    if (col_robot == nullptr)
    {
        return true;
    }

    // 2. if it belongs to other multibody, collided
    if (mModel != col_robot->mModel)
        return true;
    else
    {
        // 3. if they belong to the same multibody, begin to judge

        // 3.1 if the collision group are set to zero, then it cannot has
        // collision, return false directly
        if (mColGroup == 0 || col_robot->mColGroup == 0)
            return false;

        // std::cout << "link " << this->mLinkId << " and " <<
        // col_robot->mLinkId << " ";
        if (col_robot->mParentCollider == this ||
            this->mParentCollider == col_robot)
        {
            // std::cout << "return false\n";
            return false;
        }

        else
        {
            // std::cout << "return true\n";
            return true;
        }
    }
}

void btGenRobotCollider::ApplyForce(const tVector &force, const tVector &pos)
{
    mModel->ApplyForce(mLinkId, force, pos);
}

void btGenRobotCollider::UpdateVelocity(double dt)
{
    mModel->UpdateVelocity(dt);
}

tVector btGenRobotCollider::GetVelocityOnPoint(const tVector &pt)
{
    tMatrixXd jac;
    mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(mLinkId,
                                                        pt.segment(0, 3), jac);
    return btMathUtil::Expand(jac * mModel->Getqdot(), 0);
}

bool btGenRobotCollider::IsStatic() const { return false; }

void btGenRobotCollider::PushState(const std::string &tag,
                                   bool only_vel_and_force)
{
    mModel->PushState(tag, only_vel_and_force);
}
void btGenRobotCollider::PopState(const std::string &tag,
                                  bool only_vel_and_force)
{
    mModel->PopState(tag, only_vel_and_force);
}

void btGenRobotCollider::ClearForce() { mModel->ClearForce(); }