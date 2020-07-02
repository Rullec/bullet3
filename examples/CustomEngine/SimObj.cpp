#include "SimObj.h"
#include <iostream>
#include <fstream>
#include <limits>

#define _DEBUG_
// bool record_translate = true;
// std::string path = "record_trans.txt";
#define EXPLICIT_EULER_INTEGRATION
// #define IMPLICIT_EULER_INTEGRATION
// #define NEWTON_EULER_INTEGRATION

// boocl enable_inertia_test = false;
cSimRigidBody::tParams::tParams()
{
	// body = nullptr;
	col_shape = nullptr;
	damping = 0;
	name = "rigidbody_name";
}

cSimRigidBody::cSimRigidBody(const tParams& params) : cCollisionObject(eColObjType::Rigidbody, params.name)
{
	mTotalForce.setZero();
	mTotalTorque.setZero();
	mLinVel.setZero();
	mAngVel.setZero();
	mOrigin = params.origin;
	mLocalRot = params.rot;
	mInvInertiaLocal = params.local_inertia.cwiseInverse();
	mDamping = params.damping;
	mInvMass = params.mass < 1e-5 ? std::numeric_limits<double>::quiet_NaN() : 1.0 / params.mass;
	UpdateInertia();

	// set up flags
	m_collisionFlags &= (~btCollisionObject::CF_STATIC_OBJECT);
}

cSimRigidBody::~cSimRigidBody()
{
	// cCollisionObject::~cCollisionObject();
}
/**
 * \brief
 * \param   global_pos: applied position in global frame
 * */
void cSimRigidBody::ApplyForce(const tVector& force, const tVector& global_pos)
{
	if (IsStatic()) return;

	tVector rel_pos = global_pos - GetWorldPos();
	mTotalForce += force;
	mTotalTorque += rel_pos.cross3(force);
	// std::cout << "new total force = " << mTotalForce.transpose() << std::endl;
	// std::cout << "new total torque = " << mTotalTorque.transpose() << std::endl;
}

void cSimRigidBody::ApplyCOMForce(const tVector& force)
{
	if (IsStatic()) return;
	ApplyForce(force, GetWorldPos());
}

void cSimRigidBody::ClearForce()
{
	mTotalForce.setZero();
	mTotalTorque.setZero();
}

void cSimRigidBody::UpdateVelocity(double dt)
{
	if (IsStatic()) return;

	mTotalForce -= mDamping * mLinVel;
	mTotalTorque -= mDamping * mAngVel;

	// std::cout << "-------------------------- obj " << mName << " update vel dt = " << dt << std::endl;
	// std::cout << "old lin vel = " << mLinVel.transpose() << std::endl;
	// std::cout << "force = " << mTotalForce.transpose() << std::endl;
	mLinVel = mLinVel + mInvMass * mTotalForce * dt;
	// std::cout << "new lin vel = " << mLinVel.transpose() << std::endl;
#ifdef EXPLICIT_EULER_INTEGRATION
	// std::cout << "explicit euelr\n";
	// std::cout << "old ang vel = " << mAngVel.transpose() << std::endl;

	mAngVel = mAngVel + mInvInertia * (mTotalTorque - mAngVel.cross3(mInertia * mAngVel)) * dt;
	// std::cout << "-------------------------- obj " << mName << " update vel end" << std::endl;
	// std::cout << "new ang vel = " << mAngVel.transpose() << std::endl;
#elif defined(IMPLICIT_EULER_INTEGRATION)
	std::cout << "implicit euelr\n";
	tMatrix coef_tMatrixXd = tMatrix::Zero();
	coef_mat.block(0, 0, 3, 3) = (mInertia + dt * cMathUtil::VectorToSkewMat(mAngVel) * mInertia).block(0, 0, 3, 3).inverse();
	mAngVel = coef_tMatrixXd * (dt * mTotalTorque + mInertia * mAngVel);
#elif defined(NEWTON_EULER_INTEGRATION)
	std::cout << "single newton\n";
	SingleNewtonStep(dt);
#else
	std::cout << "NOT IMPLEMENTED\n";
	exit(0);
#endif

	// mAngVel = mAngVel + mInvInertia * mTotalTorque * dt;
	// std::cout <<"update vel, ang vel = " << mAngVel.transpose() << std::endl;
#ifdef _DEBUG_
	// std::cout << "lin vel = " << mLinVel.transpose() << std::endl;
#endif
}

void cSimRigidBody::UpdateTransform(float dt)
{
	if (IsStatic()) return;
	mOrigin = mOrigin + mLinVel * dt;
	mLocalRot = cMathUtil::AxisAngleToQuaternion(mAngVel * dt + cMathUtil::QuaternionToAxisAngle(mLocalRot));
	UpdateInertia();
}

void cSimRigidBody::UpdateInertia()
{
	mInvInertia.setZero();
	mInertia.setZero();
	mInvInertia.block(0, 0, 3, 3) = mLocalRot.toRotationMatrix() * mInvInertiaLocal.segment(0, 3).asDiagonal() * mLocalRot.toRotationMatrix().transpose();
	mInertia.block(0, 0, 3, 3) = mLocalRot.toRotationMatrix() * mInvInertiaLocal.segment(0, 3).cwiseInverse().asDiagonal() * mLocalRot.toRotationMatrix().transpose();
}

void cSimRigidBody::WriteTransAndVelToBullet()
{
	// 1. 0 order info
	btTransform trans;
	trans.setOrigin(cBulletUtil::tVectorTobtVector(mOrigin));
	trans.setRotation(cBulletUtil::tQuaternionTobtQuaternion(mLocalRot));
	setWorldTransform(trans);

	trans = getWorldTransform();
	// std::cout << "world pos = " << cBulletUtil::btVectorTotVector0(trans.getOrigin()).transpose() << std::endl;
	// 1. 1 order info
	// btRigidBody::setLinearVelocity(cBulletUtil::tVectorTobtVector(mLinVel));
	// btRigidBody::setAngularVelocity(cBulletUtil::tVectorTobtVector(mAngVel));
}

// void cSimRigidBody::ReadTransAndVelFromBullet()
// {
// 	// 0 order info
// 	btTransform trans = btRigidBody::getWorldTransform();
// 	mOrigin = cBulletUtil::btVectorTotVector0(trans.getOrigin());
// 	mLocalRot = cBulletUtil::btQuaternionTotQuaternion(trans.getRotation());

// 	// 1 order info
// 	mLinVel = cBulletUtil::btVectorTotVector0(btRigidBody::getLinearVelocity());
// 	mAngVel = cBulletUtil::btVectorTotVector0(btRigidBody::getAngularVelocity());
// }

bool cSimRigidBody::isStaticObject() const
{
	// return btCollisionObject::isStaticObject();
	return (std::isnan(mInvMass) == true);
}

bool cSimRigidBody::IsStatic() const
{
	bool is_static = isStaticObject();
	// std::cout << "rigidbody col flag = " << m_collisionFlags << ", static = " << is_static << " invmass = " << mInvMass << std::endl;
	return is_static;
}

tVector cSimRigidBody::GetTotalTorque() const
{
	return mTotalTorque;
}

/**
 * \brief			Get the velocity of a global point
 * \param pt		the position in world frame
*/
tVector cSimRigidBody::GetVelocityOnPoint(const tVector& pt)
{
	return mLinVel + mAngVel.cross3(pt - GetWorldPos());
}

// btRigidBody* cSimRigidBody::GetbtRigidBody() const
// {
// 	return static_cast<>mRigidBody;
// }
float cSimRigidBody::GetMass() const
{
	return 1.0 / mInvMass;
}
float cSimRigidBody::GetInvMass() const
{
	return mInvMass;
}
tVector cSimRigidBody::GetWorldPos() const
{
	return mOrigin;
}

tQuaternion cSimRigidBody::GetOrientation() const
{
	return mLocalRot;
}

void cSimRigidBody::SetLinVel(const tVector& v)
{
	mLinVel = v;
	mLinVel[3] = 0;
	WriteTransAndVelToBullet();
}

void cSimRigidBody::SetAngVel(const tVector& v)
{
	mAngVel = v;
	mAngVel[3] = 0;
	WriteTransAndVelToBullet();
}

void cSimRigidBody::UpdateMomentum(tVector& lin, tVector& ang)
{
	lin = GetMass() * mLinVel;
	ang = mInertia * mAngVel;
}

void cSimRigidBody::NewtonIters(double dt)
{
	int max_step = 10;
	tVector w0 = mAngVel;
	tVector func_value;
	tMatrix dfdw_num, dfdw;
	std::cout << "-----------------\n";
	for (int i = 0; i < max_step; i++)
	{
		func_value.setZero();
		dfdw.setZero();
		dfdw_num.setZero();

		EvalFuncDeri(dt, w0, dfdw);
		EvalFunc(dt, w0, func_value);

		w0.segment(0, 3).noalias() = w0.segment(0, 3) - dfdw.block(0, 0, 3, 3).inverse() * func_value.segment(0, 3);
		std::cout << "func value = " << func_value.transpose() << std::endl;
		if (func_value.norm() < 1e-10) break;
	}
	mAngVel = w0;
}

void cSimRigidBody::EvalFunc(double dt, const tVector& AngVelNew, tVector& func_value)
{
	func_value.noalias() =
		mInertia * (AngVelNew - mAngVel) +
		AngVelNew.cross3(mInertia * AngVelNew) * dt - mTotalTorque * dt;
}

void cSimRigidBody::EvalFuncDeri(double dt, const tVector& AngVelNew, tMatrix& mat)
{
	mat.noalias() =
		mInertia +
		dt * (cMathUtil::VectorToSkewMat(AngVelNew) * mInertia -
			  cMathUtil::VectorToSkewMat(mInertia * AngVelNew));
}

void cSimRigidBody::EvalFuncDeriNumerically(double dt, const tVector& AngVelNew, tMatrix& mat)
{
	tVector x = AngVelNew;
	double threshold = 1e-5;
	tVector func_big, func_raw;
	tMatrixXd tMatrixXd = tMatrix::Zero();
	for (int i = 0; i < 3; i++)
	{
		x[i] += threshold;
		EvalFunc(dt, x, func_big);
		x[i] -= threshold;
		EvalFunc(dt, x, func_raw);
		mat.row(i) = (func_big - func_raw) / threshold;
	}
}

tMatrix cSimRigidBody::GetInvInertia() const
{
	return mInvInertia;
}

tMatrix cSimRigidBody::GetInertia() const
{
	return mInertia;
}

tVector cSimRigidBody::GetLinVel() const
{
	return mLinVel;
}

tVector cSimRigidBody::GetAngVel() const
{
	return mAngVel;
}

void cSimRigidBody::PushState()
{
	mOldState.mAngVel = mAngVel;
	mOldState.mDamping = mDamping;
	mOldState.mInertia = mInertia;
	mOldState.mInvInertia = mInvInertia;
	mOldState.mInvInertiaLocal = mInvInertiaLocal;
	mOldState.mInvMass = mInvMass;
	mOldState.mLinVel = mLinVel;
	mOldState.mLocalRot = mLocalRot;
	mOldState.mOrigin = mOrigin;
	mOldState.mTotalForce = mTotalTorque;
	mOldState.mTotalTorque = mTotalTorque;
}
void cSimRigidBody::PopState()
{
	mAngVel = mOldState.mAngVel;
	mDamping = mOldState.mDamping;
	mInertia = mOldState.mInertia;
	mInvInertia = mOldState.mInvInertia;
	mInvInertiaLocal = mOldState.mInvInertiaLocal;
	mInvMass = mOldState.mInvMass;
	mLinVel = mOldState.mLinVel;
	mLocalRot = mOldState.mLocalRot;
	mOrigin = mOldState.mOrigin;
	mTotalForce = mOldState.mTotalTorque;
	mTotalTorque = mOldState.mTotalTorque;
}