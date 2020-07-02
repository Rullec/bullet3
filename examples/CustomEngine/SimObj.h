#include <memory>
#include "../ExampleBrowser/ID_test/BulletUtil.h"
#include "btBulletDynamicsCommon.h"
#include "ColObjBase.h"

// class cSimRigidBody : public std::enable_shared_from_this<cSimRigidBody>
class cSimRigidBody : public cCollisionObject
{
public:
	struct tParams
	{
		tParams();
		// btRigidBody::btRigidBodyConstructionInfo info;
		// btRigidBody *body;
		tVector origin;
		tQuaternion rot;
		tVector local_inertia;
		double mass;
		btCollisionShape *col_shape;
		std::string name;
		float damping;
	};
	cSimRigidBody(const tParams &params);
	virtual ~cSimRigidBody();

	// apply force
	virtual void ApplyForce(const tVector &force, const tVector &) override final;
	void ApplyCOMForce(const tVector &force);
	void ClearForce();

	virtual void UpdateVelocity(double dt) override;
	void UpdateTransform(float dt);
	void UpdateMomentum(tVector &lin, tVector &ang);

	void WriteTransAndVelToBullet();
	// void ReadTransAndVelFromBullet();

	// btRigidBody *GetbtRigidBody() const;
	float GetMass() const;
	float GetInvMass() const;
	tMatrix GetInvInertia() const;
	tMatrix GetInertia() const;
	tVector GetWorldPos() const;
	tQuaternion GetOrientation() const;
	tVector GetLinVel() const;
	tVector GetAngVel() const;
	tVector GetTotalTorque() const;
	virtual tVector GetVelocityOnPoint(const tVector &pt) override final;
	void SetLinVel(const tVector &v);
	void SetAngVel(const tVector &v);

	virtual bool IsStatic() const override;
	bool isStaticObject() const;
	virtual void PushState() override;
	virtual void PopState() override;

protected:
	// btRigidBody *mRigidBody;

	// External forces
	tVector mTotalForce;
	tVector mTotalTorque;

	// state vars
	tVector mLinVel;
	tVector mAngVel;
	tQuaternion mLocalRot;
	tVector mOrigin;

	// Mass vars
	float mInvMass;
	tMatrix mInvInertia, mInertia;
	tVector mInvInertiaLocal;

	// other stuff
	float mDamping;
	void UpdateInertia();

	// updating the euler equation by single step Newton method
	void NewtonIters(double dt);
	// void EvalFunc(double dt, const tVector& AngVelNew, const tVector& AngVelOld, tVector & func_value);
	void EvalFunc(double dt, const tVector &AngVelNew, tVector &func_value);
	void EvalFuncDeri(double dt, const tVector &AngVelNew, tMatrix &mat);
	void EvalFuncDeriNumerically(double dt, const tVector &AngVelNew, tMatrix &mat);

	struct
	{
		tVector mTotalForce;
		tVector mTotalTorque;

		// state vars
		tVector mLinVel;
		tVector mAngVel;
		tQuaternion mLocalRot;
		tVector mOrigin;

		// Mass vars
		float mInvMass;
		tMatrix mInvInertia, mInertia;
		tVector mInvInertiaLocal;

		// other stuff
		float mDamping;
	} mOldState;
};