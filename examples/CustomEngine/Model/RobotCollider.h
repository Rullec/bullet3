#pragma once
// #include "BulletCollision/CollisionDispatch/btCollisionObject.h"
// #include "BulletDynamics/Dynamics/btRigidBody.h"
#include "../ColObjBase.h"

class cRobotModelDynamics;
// this class is typically used to avoid the collision between parent link and its child.
struct cRobotCollider : public cCollisionObject
{
public:
	cRobotCollider(cRobotModelDynamics* model, int link_id, const std::string& name);
	virtual ~cRobotCollider();

	static cRobotCollider* upcast(btCollisionObject* colObj);
	static const cRobotCollider* upcast(const btCollisionObject* colObj);

	virtual bool checkCollideWithOverride(const btCollisionObject* co) const override;

	virtual void ApplyForce(const tVector& force, const tVector& pos) override final;
	virtual void ClearForce() override final;
	virtual void UpdateVelocity(double dt) override final;
	virtual tVector GetVelocityOnPoint(const tVector& pt) override final;
	virtual bool IsStatic() const override final;
	virtual void PushState(const std::string & tag, bool only_vel_and_force = false) override final;
	virtual void PopState(const std::string& tag, bool only_vel_and_force = false) override final;

	cRobotModelDynamics* mModel;
	cRobotCollider* mParentCollider;
	int mLinkId;
};