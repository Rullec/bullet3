#include "RobotCollider.h"
#include <iostream>
#include "RobotModel.h"

cRobotCollider::cRobotCollider(cRobotModel* model, int link_id, const std::string& name)
	: mModel(model), mLinkId(link_id), cCollisionObject(eColObjType::RobotCollder, name)
{
	m_checkCollideWith = true;
	m_collisionFlags &= (~btCollisionObject::CF_STATIC_OBJECT);
	m_internalType = CO_COLLISION_OBJECT;
}

cRobotCollider::~cRobotCollider()
{
}

cRobotCollider* cRobotCollider::upcast(btCollisionObject* colObj)
{
	return dynamic_cast<cRobotCollider*>(colObj);
}
const cRobotCollider* cRobotCollider::upcast(const btCollisionObject* colObj)
{
	return dynamic_cast<const cRobotCollider*>(colObj);
}

bool cRobotCollider::checkCollideWithOverride(const btCollisionObject* co) const
{
	// std::cout << "checkCollideWithOverride called\n";
	// std::cout <<"multibody col flag = " << m_collisionFlags << std::endl;
	const cRobotCollider* col_robot = cRobotCollider::upcast(co);

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
		// std::cout << "link " << this->mLinkId << " and " << col_robot->mLinkId << " ";
		if (col_robot->mParentCollider == this || this->mParentCollider == col_robot)
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

void cRobotCollider::ApplyForce(const tVector& force, const tVector& pos)
{
	mModel->ApplyForce(mLinkId, force, pos);
}

void cRobotCollider::UpdateVelocity(double dt)
{
	mModel->UpdateVelocity(dt);
}

tVector cRobotCollider::GetVelocityOnPoint(const tVector& pt)
{
	tMatrixXd jac;
	mModel->ComputeJacobiByGivenPointTotalDOFWorldFrame(mLinkId, pt.segment(0, 3), jac);
	return cMathUtil::Expand(jac * mModel->Getqdot(), 0);
}

bool cRobotCollider::IsStatic() const
{
	return false;
}

void cRobotCollider::PushState()
{
	mModel->PushState();
}
void cRobotCollider::PopState()
{
	mModel->PopState();
}