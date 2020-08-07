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

extern int global_frame_id;
// extern std::string gOutputLogPath;
// boocl enable_inertia_test = false;
cRigidBody::tParams::tParams()
{
	// body = nullptr;
	col_shape = nullptr;
	damping = 0;
	name = "rigidbody_name";
}

cRigidBody::cRigidBody(const tParams& params) : cCollisionObject(eColObjType::Rigidbody, params.name)
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

cRigidBody::~cRigidBody()
{
	// cCollisionObject::~cCollisionObject();
}
/**
 * \brief
 * \param   global_pos: applied position in global frame
 * */
void cRigidBody::ApplyForce(const tVector& force, const tVector& global_pos)
{
	if (IsStatic()) return;

	tVector rel_pos = global_pos - GetWorldPos();
	mTotalForce += force;
	mTotalTorque += rel_pos.cross3(force);
	// std::cout << "simobj apply force = " << force.transpose() << std::endl;
	// std::cout << "new total torque = " << mTotalTorque.transpose() << std::endl;
	// std::cout << "new total force = " << mTotalForce.transpose() << std::endl;
}

void cRigidBody::ApplyCOMForce(const tVector& force)
{
	if (IsStatic()) return;
	ApplyForce(force, GetWorldPos());
}

void cRigidBody::ClearForce()
{
	mTotalForce.setZero();
	mTotalTorque.setZero();
}

void cRigidBody::UpdateVelocity(double dt)
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

	// if (this->IsStatic() == false)
	// {
	// 	std::ofstream fout(gOutputLogPath, std::ios::app);
	// 	// frameid, pos, rot, ang, vel
	// 	fout << "----------------frame id " << global_frame_id << "--------------------\n";
	// 	auto link = this;
	// 	fout << "world pos = " << link->GetWorldPos().transpose() << std::endl;
	// 	fout << "world rot = \n"
	// 		 << link->GetOrientation().coeffs().transpose() << std::endl;
	// 	fout << "ang vel = " << link->mAngVel.transpose() << std::endl;
	// 	fout << "lin vel = " << (link->mLinVel).transpose() << std::endl;
	// 	fout.close();
	// }
}

void cRigidBody::UpdateTransform(float dt)
{
	if (IsStatic()) return;
	mOrigin = mOrigin + mLinVel * dt;
	mLocalRot = btMathUtil::AxisAngleToQuaternion(mAngVel * dt + btMathUtil::QuaternionToAxisAngle(mLocalRot));
	UpdateInertia();
}

void cRigidBody::UpdateInertia()
{
	mInvInertia.setZero();
	mInertia.setZero();
	mInvInertia.block(0, 0, 3, 3) = mLocalRot.toRotationMatrix() * mInvInertiaLocal.segment(0, 3).asDiagonal() * mLocalRot.toRotationMatrix().transpose();
	mInertia.block(0, 0, 3, 3) = mLocalRot.toRotationMatrix() * mInvInertiaLocal.segment(0, 3).cwiseInverse().asDiagonal() * mLocalRot.toRotationMatrix().transpose();
}

void cRigidBody::WriteTransAndVelToBullet()
{
	// 1. 0 order info
	btTransform trans;
	trans.setOrigin(btBulletUtil::tVectorTobtVector(mOrigin));
	trans.setRotation(btBulletUtil::tQuaternionTobtQuaternion(mLocalRot));
	setWorldTransform(trans);

	trans = getWorldTransform();
	// std::cout << "world pos = " << btBulletUtil::btVectorTotVector0(trans.getOrigin()).transpose() << std::endl;
	// 1. 1 order info
	// btRigidBody::setLinearVelocity(btBulletUtil::tVectorTobtVector(mLinVel));
	// btRigidBody::setAngularVelocity(btBulletUtil::tVectorTobtVector(mAngVel));
}

// void cSimRigidBody::ReadTransAndVelFromBullet()
// {
// 	// 0 order info
// 	btTransform trans = btRigidBody::getWorldTransform();
// 	mOrigin = btBulletUtil::btVectorTotVector0(trans.getOrigin());
// 	mLocalRot = btBulletUtil::btQuaternionTotQuaternion(trans.getRotation());

// 	// 1 order info
// 	mLinVel = btBulletUtil::btVectorTotVector0(btRigidBody::getLinearVelocity());
// 	mAngVel = btBulletUtil::btVectorTotVector0(btRigidBody::getAngularVelocity());
// }

bool cRigidBody::isStaticObject() const
{
	// return btCollisionObject::isStaticObject();
	return (std::isnan(mInvMass) == true);
}

bool cRigidBody::IsStatic() const
{
	bool is_static = isStaticObject();
	// std::cout << "rigidbody col flag = " << m_collisionFlags << ", static = " << is_static << " invmass = " << mInvMass << std::endl;
	return is_static;
}

tVector cRigidBody::GetTotalTorque() const
{
	return mTotalTorque;
}

/**
 * \brief			Get the velocity of a global point
 * \param pt		the position in world frame
*/
tVector cRigidBody::GetVelocityOnPoint(const tVector& pt)
{
	return mLinVel + mAngVel.cross3(pt - GetWorldPos());
}

// btRigidBody* cSimRigidBody::GetbtRigidBody() const
// {
// 	return static_cast<>mRigidBody;
// }
float cRigidBody::GetMass() const
{
	return 1.0 / mInvMass;
}
float cRigidBody::GetInvMass() const
{
	return mInvMass;
}
tVector cRigidBody::GetWorldPos() const
{
	return mOrigin;
}

tQuaternion cRigidBody::GetOrientation() const
{
	return mLocalRot;
}

void cRigidBody::SetLinVel(const tVector& v)
{
	mLinVel = v;
	mLinVel[3] = 0;
	WriteTransAndVelToBullet();
}

void cRigidBody::SetAngVel(const tVector& v)
{
	mAngVel = v;
	mAngVel[3] = 0;
	WriteTransAndVelToBullet();
}

void cRigidBody::UpdateMomentum(tVector& lin, tVector& ang)
{
	lin = GetMass() * mLinVel;
	ang = mInertia * mAngVel;
}

void cRigidBody::NewtonIters(double dt)
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

void cRigidBody::EvalFunc(double dt, const tVector& AngVelNew, tVector& func_value)
{
	func_value.noalias() =
		mInertia * (AngVelNew - mAngVel) +
		AngVelNew.cross3(mInertia * AngVelNew) * dt - mTotalTorque * dt;
}

void cRigidBody::EvalFuncDeri(double dt, const tVector& AngVelNew, tMatrix& mat)
{
	mat.noalias() =
		mInertia +
		dt * (btMathUtil::VectorToSkewMat(AngVelNew) * mInertia -
			  btMathUtil::VectorToSkewMat(mInertia * AngVelNew));
}

void cRigidBody::EvalFuncDeriNumerically(double dt, const tVector& AngVelNew, tMatrix& mat)
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

tMatrix cRigidBody::GetInvInertia() const
{
	return mInvInertia;
}

tMatrix cRigidBody::GetInertia() const
{
	return mInertia;
}

tVector cRigidBody::GetLinVel() const
{
	return mLinVel;
}

tVector cRigidBody::GetAngVel() const
{
	return mAngVel;
}

void cRigidBody::PushState(const std::string& tag, bool only_vel_and_force)
{
	if (mStateStack.size() >= mStackLimit)
	{
		std::cout << "[error] cSimRigidBody " << mName << " Stack try to push " << tag << " but it is full\n";
		exit(1);
	}
	tStateRecord* state = new tStateRecord();

	state->OnlyVelocityForceRecord = only_vel_and_force;
	state->mAngVel = mAngVel;
	state->mLinVel = mLinVel;
	state->mDamping = mDamping;
	state->mTotalForce = mTotalForce;
	state->mTotalTorque = mTotalTorque;
	if (only_vel_and_force == false)
	{
		state->mInertia = mInertia;
		state->mInvInertia = mInvInertia;
		state->mInvInertiaLocal = mInvInertiaLocal;
		state->mInvMass = mInvMass;
		state->mLocalRot = mLocalRot;
		state->mOrigin = mOrigin;
	}

	// std::cout <<"push force = " << mTotalForce.transpose() << std::endl;
	mStateStack.push_back(std::make_pair(tag, state));
}
void cRigidBody::PopState(const std::string& tag, bool only_vel_and_force)
{
	if (mStateStack.size() == 0)
	{
		std::cout << "[error] RigidBody" << mName << " stack is empty when you try to pop " << tag << std::endl;
		exit(1);
	}
	else if (mStateStack.back().first != tag)
	{
		std::cout << "[error] RigidBody " << mName << "stack trying to pop " << mStateStack.back().first << " but the user try to do " << tag << std::endl;
		exit(1);
	}
	auto state = mStateStack.back().second;
	if (state->OnlyVelocityForceRecord != only_vel_and_force)
	{
		std::cout << "[error] RigidBody " << mName << "stack trying to pop vel " << state->OnlyVelocityForceRecord << " but the user try to do " << only_vel_and_force << std::endl;
		exit(1);
	}

	mLinVel = state->mLinVel;
	mDamping = state->mDamping;
	mAngVel = state->mAngVel;
	mTotalForce = state->mTotalForce;
	mTotalTorque = state->mTotalTorque;
	if (only_vel_and_force == false)
	{
		mInertia = state->mInertia;
		mInvInertia = state->mInvInertia;
		mInvInertiaLocal = state->mInvInertiaLocal;
		mInvMass = state->mInvMass;

		mLocalRot = state->mLocalRot;
		mOrigin = state->mOrigin;
	}

	// std::cout <<"pop force = " << mTotalForce.transpose() << std::endl;
	delete state;
	mStateStack.pop_back();
}