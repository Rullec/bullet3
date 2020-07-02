#pragma once
// #include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include <string>
#include "../ExampleBrowser/ID_test/MathUtil.h"

enum eColObjType
{
	RobotCollder,
	Rigidbody
};

class cCollisionObject : virtual public btCollisionObject
{
public:
	cCollisionObject(eColObjType type, const std::string &name);
	eColObjType GetType();
	const std::string &GetName();

	virtual void ApplyForce(const tVector &force, const tVector &pos) = 0;
	virtual void UpdateVelocity(double dt) = 0;
	virtual tVector GetVelocityOnPoint(const tVector &pt) = 0;
	virtual bool IsStatic() const = 0;
	virtual void PushState() = 0;
	virtual void PopState() = 0;
protected:
	eColObjType mType;
	std::string mName;
};