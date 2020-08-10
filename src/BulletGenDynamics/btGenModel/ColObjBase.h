#pragma once
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
// #include "src/BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletGenDynamics/btGenUtil/MathUtil.h"
#include <string>


enum eColObjType
{
	RobotCollder,
	Rigidbody
};

class btGenCollisionObject : public btCollisionObject
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	btGenCollisionObject(eColObjType type, const std::string &name);
	eColObjType GetType();
	const std::string &GetName();

	virtual void ApplyForce(const tVector &force, const tVector &pos) = 0;
	virtual void UpdateVelocity(double dt) = 0;
	virtual void ClearForce() = 0;
	virtual tVector GetVelocityOnPoint(const tVector &pt) = 0;
	virtual bool IsStatic() const = 0;
	virtual void PushState(const std::string &tag, bool only_vel_and_force = false) = 0;
	virtual void PopState(const std::string &tag, bool only_vel_and_force = false) = 0;

protected:
	eColObjType mType;
	std::string mName;
};