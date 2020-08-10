#pragma once
// #include "BulletCollision/CollisionDispatch/btCollisionObject.h"
// #include "BulletDynamics/Dynamics/btRigidBody.h"
#include "ColObjBase.h"

class cRobotModelDynamics;
// this class is typically used to avoid the collision between parent link and its child.
struct btGenRobotCollider : public btGenCollisionObject
{
public:
	btGenRobotCollider(cRobotModelDynamics* model, int link_id, const std::string& name, int col_group);
	virtual ~btGenRobotCollider();

	static btGenRobotCollider* upcast(btCollisionObject* colObj);
	static const btGenRobotCollider* upcast(const btCollisionObject* colObj);

	virtual bool checkCollideWithOverride(const btCollisionObject* co) const override;

	virtual void ApplyForce(const tVector& force, const tVector& pos) override final;
	virtual void ClearForce() override final;
	virtual void UpdateVelocity(double dt) override final;
	virtual tVector GetVelocityOnPoint(const tVector& pt) override final;
	virtual bool IsStatic() const override final;
	virtual void PushState(const std::string & tag, bool only_vel_and_force = false) override final;
	virtual void PopState(const std::string& tag, bool only_vel_and_force = false) override final;

	cRobotModelDynamics* mModel;
	btGenRobotCollider* mParentCollider;
	int mLinkId;
	int mColGroup;
};