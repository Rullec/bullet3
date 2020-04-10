#include "ConstraintMultibody.hpp"
#include <cmath>
#include "../ExampleBrowser/ID_test/BulletUtil.h"
#include <iostream>

/*
	@funciton: cConstraintMultibody
	@params: num_links Type int, the number of links in this chain
	@params: floating_base Type bool, enable floating base or not?
	@params: spherical_joint Type bool, enable spherical or not?
*/
cConstraintMultibody::cConstraintMultibody(
    btDiscreteDynamicsWorld * world,
    btAlignedObjectArray<btCollisionShape*> & col_shapes, 
    int num_links,
    bool floating_base,
    bool spherical_joint):mWorld(world)
{
    assert(num_links > 0);
    mNumLinks = num_links;
    mLinkLst.resize(mNumLinks);

    // 1. init all values
	btRigidBody * parent_link = nullptr,
				* child_link = nullptr;
	btVector3 box_size = btVector3(1, 1, 5);
	btCollisionShape* shape = new btBoxShape(box_size); // It's better to use shared box shape
	col_shapes.push_back(shape);
	double length = box_size[2] * 2;
	double mass = 1.0f;
	btTransform tr;

	btVector3 axisA(0.f, 1.f, 0.f);
	btVector3 axisB(0.f, 1.f, 0.f);
	btVector3 pivotA(0.f, 0.f, length / 2);
	btVector3 pivotB(0.f, 0.f, -length / 2);

    btHingeConstraint* hinge_cons = NULL;
    btPoint2PointConstraint * p2p_cons = NULL;
	// 2. create all links recursively

	// 2.1 create base link
	tr.setIdentity();	
	tr.setOrigin(btVector3(btScalar(0), btScalar(0), btScalar(0)));
	if(true == floating_base)
		parent_link = createRigidBody(0.0f, tr, shape);
	else
		parent_link = createRigidBody(mass, tr, shape);
    mLinkLst[0] = parent_link;
    // parent_link->setLinearVelocity(btVector3(10, 10, 10));
    parent_link->setAngularVelocity(btVector3(10, 10, 10));
    parent_link->setDamping(0, 0);
    // parent_link->setFlags(btRigidBodyFlags::BT_DISABLE_WORLD_GRAVITY);
    // parent_link->setFlags(btRigidBodyFlags::BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT);
    // parent_link->setFlags(btRigidBodyFlags::BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY);
    parent_link->setFlags(btRigidBodyFlags::BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD);

    // parent_link->getAngularDamping()
	// 2.2 create child links
	for(int i=0; i< num_links-1; i++)
	{
		// set up child link
		tr.setIdentity();
		tr.setOrigin(btVector3(btScalar(0), btScalar(0), btScalar((i + 1) * length)));
		child_link = createRigidBody(1.0f, tr, shape);
        mLinkLst[i+1] = child_link;

		// child_link->setActivationState(DISABLE_DEACTIVATION);

		// set up constraint
		if(spherical_joint == false)
		{
		hinge_cons = new btHingeConstraint(*parent_link, *child_link, pivotA, pivotB, axisA, axisB);
		world->addConstraint(hinge_cons, true);
		}
		else
		{
			p2p_cons = new btPoint2PointConstraint(*parent_link, *child_link, pivotA, pivotB);
			world->addConstraint(p2p_cons, true);
		}

		parent_link = child_link;
		child_link = nullptr;
	}
}

btVector3 cConstraintMultibody::GetLinkWorldPos(int id)
{
    assert(id < mNumLinks);
    return mLinkLst[id]->getCenterOfMassPosition();
}

// get the global orientation for this link as a quaternion
// it can rotate a vector in local frame to world frame
btQuaternion cConstraintMultibody::GetLinkWorldRot(int id)
{
    assert(id < mNumLinks);
    return mLinkLst[id]->getOrientation();
}

btVector3 cConstraintMultibody::GetLinkWorldVel(int id)
{
    assert(id < mNumLinks);
    return mLinkLst[id]->getLinearVelocity();
}

btVector3 cConstraintMultibody::GetLinkWorldOmega(int id)
{
    assert(id < mNumLinks);
    return mLinkLst[id]->getAngularVelocity();
}

double cConstraintMultibody::GetLinkMass(int id)
{
    assert(id < mNumLinks && id >= 0);
    double mass = 1.0 / mLinkLst[id]->getInvMass();
    assert(std::isinf(mass) == false);
    return mass;
}

btVector3 cConstraintMultibody::GetCOM()
{
    btVector3 COM = btVector3(0, 0, 0);
    for(int i=0; i<mNumLinks; i++)
    {
        btRigidBody * cur_body = mLinkLst[i];
        double mass = 1.0 / cur_body->getInvMass();
        assert(std::isinf(mass) == false);
        COM += mass * GetLinkWorldPos(i);
    }
    return COM;
}

btVector3 cConstraintMultibody::CalcLinearMomentum()
{
    btVector3 total_mom = btVector3(0, 0, 0);

    for(int i=0; i<mNumLinks; i++)
    {
        btRigidBody * cur_body = mLinkLst[i];
        double mass = 1.0 / cur_body->getInvMass();
        assert(std::isinf(mass) == false);
        total_mom += mass * GetLinkWorldVel(i);
    }

    return total_mom;
}

double cConstraintMultibody::GetTotalMass()
{
    double total_mass = 0;
    for(int i=0; i<mNumLinks; i++)
    {
        double mass = 1.0 / mLinkLst[i]->getInvMass();
        assert(std::isinf(mass) == false);
        total_mass += mass;
    }
    return total_mass;
}

btRigidBody * cConstraintMultibody::GetLink(int id)
{
    assert(id < mNumLinks);
    return mLinkLst[id];
}
// Calculate angualr momentum for multibody system referred to its COM
// L = \sum_i R_i * I_body_i * R_i^T * omega
btVector3 cConstraintMultibody::CalcAngMomentum()
{
    btMatrix3x3 rot_i, inertia;
    btVector3 omega, COM = GetCOM(), total_ang_mom = btVector3(0, 0, 0);
    for(int i=0; i<mNumLinks; i++)
    {
        auto cur_link = GetLink(i);
        rot_i = cur_link->getWorldTransform().getBasis();
        // std::cout <<"link " << i <<" rot mat = \n" << cBulletUtil::btMatrixTotMatrix0(rot_i) << std::endl;
        omega = GetLinkWorldOmega(i);
        btVector3 part1 = btVector3(0, 0, 0);
        // btVector3 part1 = (GetLinkWorldPos(i) - COM).cross(GetLinkWorldVel(i)) * GetLinkMass(i);
        btVector3 part2;
        inertia = cBulletUtil::AsDiagnoal(mLinkLst[i]->getLocalInertia());
        // tVector inertia_vec = cBulletUtil::btVectorTotVector0();
        // std::cout <<"link " << i <<" inertia vec = " << inertia_vec.transpose() << std::endl;
        part2 = rot_i * inertia * rot_i.transpose() * omega;
        // total_ang_mom +=  + rot_i.scaled(mLinkLst[i]->getLocalInertia()) * rot_i.transpose() * omega;
        std::cout <<"raw omega = " << cBulletUtil::btVectorTotVector0(omega).transpose() << std::endl;
        // std::cout <<"new 1 omega = " << cBulletUtil::btVectorTotVector0(rot_i * omega).transpose() << std::endl;
        // std::cout <<"new 2 omega = " << cBulletUtil::btVectorTotVector0(rot_i.transpose() * omega).transpose() << std::endl;
        total_ang_mom += part1 + part2;
    }

    // exit(1);
    return total_ang_mom;
}

btRigidBody* cConstraintMultibody::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, const btVector4& color)
{
		btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			shape->calculateLocalInertia(mass, localInertia);

			//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);

		btRigidBody* body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
		btRigidBody* body = new btRigidBody(mass, 0, shape, localInertia);
		body->setWorldTransform(startTransform);
#endif  //

		body->setUserIndex(-1);
		mWorld->addRigidBody(body);
		return body;
}