#include "cCollisionWorld.h"
#include "LinearMath/btIDebugDraw.h"
#include "btMultiBodyDynamicsWorld.h"
#include "btMultiBodyConstraintSolver.h"
#include "btMultiBody.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btQuickprof.h"
#include "btMultiBodyConstraint.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btSerializer.h"
#include "btMultiBodyDynamicsWorldUtil.h"
#include <iostream>
#define BULLET_GENERATE_LIB
#ifndef BULLET_GENERATE_LIB
#include "BulletGenDynamics/btGenUtil/BulletUtil.h"
#endif
#define DISABLE_SCREEN_LOG 
#ifdef BULLET_GENERATE_LIB
#define DISABLE_FILE_LOG
#endif
extern bool gDisableDeactivation;

cCollisionWorld::cCollisionWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btMultiBodyConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration):
	btMultiBodyDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
{
	std::cout << "[debug] cCollisionWorld::cCollisionWorld constructed\n";

	mCurFrame = 0;
	mWorldLog.open("regular_debug.log");
	mWorldLog << "";

#ifndef BT_USE_VIRTUAL_CLEARFORCES_AND_GRAVITY
	std::cout << "[error] There should be another logic, exit\n";
	exit(0);
#endif // !BT_USE_VIRTUAL_CLEARFORCES_AND_GRAVITY
}

cCollisionWorld::~cCollisionWorld()
{
	std::cout << "[debug] cCollisionWorld::cCollisionWorld deconstructed\n";
}

int cCollisionWorld::stepSimulation(btScalar timeStep, int maxSubSteps/* = 1*/, btScalar fixedTimeStep/* = btScalar(1.) / btScalar(60.)*/)
{
	mCurFrame++;
	// ����ʱ�䲽����substep�������̶�timestep
	if (maxSubSteps != 0 || std::abs(timeStep - fixedTimeStep) > 1e-10)
	{
		std::cout << "[error] cCollisionWorld::stepSimulation unsupported setting\n";
		exit(0);
	}
	
	startProfiling(timeStep);

	//process some debugging flags
	if (getDebugDrawer())
	{
		btIDebugDraw* debugDrawer = getDebugDrawer();
		gDisableDeactivation = (debugDrawer->getDebugMode() & btIDebugDraw::DBG_NoDeactivation) != 0;
	}

	// ���ݵ�ǰ������̬����������ٶȣ�������ٶȺ����ٶȣ����Ҹ�ֵ��"interpolation"���ݽṹ�С�
	//saveKinematicState(timeStep);

	// ��������������link����������ȥ��
	applyGravity();

	// һ��timestep
	internalSingleStepSimulation(timeStep);

	// �����?
	clearForces();

#ifndef BT_NO_PROFILE
	CProfileManager::Increment_Frame_Counter();
#endif  //BT_NO_PROFILE

	return 0;
}


void cCollisionWorld::internalSingleStepSimulation(btScalar timeStep)
{
	BT_PROFILE("internalSingleStepSimulation");

	if (0 != m_internalPreTickCallback)
	{
		(*m_internalPreTickCallback)(this, timeStep);
	}

	/* 
		��ÿһ�����壬ʩ������(ֱ��˥���ٶ�)
		ע�⣬��multibodyû��Ӱ��
	*/
	predictUnconstraintMotion(timeStep);

	btDispatcherInfo& dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	// ����Ԥ���ԽӴ�
	createPredictiveContacts(timeStep);

	// ��ײ���?
	performDiscreteCollisionDetection();

	// �������?
	calculateSimulationIslands();

	// ��ȡ�Ӵ���Ϣ
	getSolverInfo().m_timeStep = timeStep;

	// ���Ӵ�������Լ��(����joint limitԼ��): 
	/*
		���ģ�������Ǹ�����ײ�㴦�ĳ������������������У�Ҳ�Ѿ����ٶȸ����ˡ�?
	*/
	solveConstraints(getSolverInfo());

	// λ�ƻ��֣������Ѿ���������յ��ٶȣ����Ҹ������ٶȡ����ڿ�ʼ���ǻ���λ�ơ�?
	integrateTransforms(timeStep);

	// ����: �������ײ����й�ϵ����Ҫ��...
	updateActivationState(timeStep);

	if (0 != m_internalTickCallback)
	{
		(*m_internalTickCallback)(this, timeStep);
	}
}

void cCollisionWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	forwardKinematics();
	clearMultiBodyConstraintForces();

	// init cons
	solveCons1_ConstraintSort(solverInfo);

	// apply active force
	solveCons2_ApplyActiveForce(solverInfo);

	// solve the impulse for each constraints, then update deltaV
	solveCons3_solveCons(solverInfo);

	// update velocity m_real_buf
	solveCons4_finalize(solverInfo);
	
}

void cCollisionWorld::solveCons1_ConstraintSort(btContactSolverInfo& solverInfo)
{
	{
		m_sortedConstraints.resize(m_constraints.size());
		int i;
		for (i = 0; i < getNumConstraints(); i++)
		{
			m_sortedConstraints[i] = m_constraints[i];
		}
		m_sortedConstraints.quickSort(btSortConstraintOnIslandPredicate2());
		btTypedConstraint** constraintsPtr = getNumConstraints() ? &m_sortedConstraints[0] : 0;

		m_sortedMultiBodyConstraints.resize(m_multiBodyConstraints.size());
		for (i = 0; i < m_multiBodyConstraints.size(); i++)
		{
			m_sortedMultiBodyConstraints[i] = m_multiBodyConstraints[i];
		}
		m_sortedMultiBodyConstraints.quickSort(btSortMultiBodyConstraintOnIslandPredicate());

		btMultiBodyConstraint** sortedMultiBodyConstraints = m_sortedMultiBodyConstraints.size() ? &m_sortedMultiBodyConstraints[0] : 0;

		m_solverMultiBodyIslandCallback->setup(&solverInfo, constraintsPtr, m_sortedConstraints.size(), sortedMultiBodyConstraints, m_sortedMultiBodyConstraints.size(), getDebugDrawer());

		auto num_objs = getCollisionWorld()->getNumCollisionObjects(), manifolds = getCollisionWorld()->getDispatcher()->getNumManifolds();

		// ��仰����ʲôҲû��?
		m_constraintSolver->prepareSolve(num_objs, manifolds);
	}

}

void cCollisionWorld::solveCons2_ApplyActiveForce(btContactSolverInfo& solverInfo)
{

	// ��������������
	{
		//PrintMultibodyInfo("before active force");

		BT_PROFILE("btMultiBody stepVelocities");
		for (int i = 0; i < this->m_multiBodies.size(); i++)
		{
			btMultiBody* bod = m_multiBodies[i];

			bool isSleeping = false;

			if (bod->getBaseCollider() && bod->getBaseCollider()->getActivationState() == ISLAND_SLEEPING)
			{
				isSleeping = true;
			}
			for (int b = 0; b < bod->getNumLinks(); b++)
			{
				if (bod->getLink(b).m_collider && bod->getLink(b).m_collider->getActivationState() == ISLAND_SLEEPING)
					isSleeping = true;
			}

			if (!isSleeping)
			{
				//useless? they get resized in stepVelocities once again (AND DIFFERENTLY)
				m_scratch_r.resize(bod->getNumLinks() + 1);  //multidof? ("Y"s use it and it is used to store qdd)
				m_scratch_v.resize(bod->getNumLinks() + 1);
				m_scratch_m.resize(bod->getNumLinks() + 1);
				bool doNotUpdatePos = false;
				bool isConstraintPass = false;
				{
					if (!bod->isUsingRK4Integration())
					{
						bod->computeAccelerationsArticulatedBodyAlgorithmMultiDof(solverInfo.m_timeStep,
							m_scratch_r, m_scratch_v, m_scratch_m, isConstraintPass,
							getSolverInfo().m_jointFeedbackInWorldSpace,
							getSolverInfo().m_jointFeedbackInJointFrame);
					}
					else
					{
						std::cout << "[error] cCollisionWorld:: there should be another logic\n";
						exit(0);
					}
				}

#ifndef BT_USE_VIRTUAL_CLEARFORCES_AND_GRAVITY
				bod->clearForcesAndTorques();
#endif         //BT_USE_VIRTUAL_CLEARFORCES_AND_GRAVITY
			}  //if (!isSleeping)
		}
		//PrintMultibodyInfo("after active force");

	}

}

void cCollisionWorld::solveCons3_solveCons(btContactSolverInfo& solverInfo)
{
	m_islandManager->buildAndProcessIslands(getCollisionWorld()->getDispatcher(), getCollisionWorld(), m_solverMultiBodyIslandCallback);

	m_solverMultiBodyIslandCallback->processConstraints();

	m_constraintSolver->allSolved(solverInfo, m_debugDrawer);
}

void cCollisionWorld::solveCons4_finalize(btContactSolverInfo& solverInfo)
{
	if (m_multiBodies[0]->internalNeedsJointFeedback() == true)
	{
		std::cout << "[error] void cCollisionWorld::solveCons4_finalize: unsupported joint feedback\n";
		exit(0);
	}

	//PrintMultibodyInfo("before final update", true);
	for (int i = 0; i < this->m_multiBodies.size(); i++)
	{
		btMultiBody* bod = m_multiBodies[i];
		bod->processDeltaVeeMultiDof2();
	}
	//PrintMultibodyInfo("after final update", true);
}

void cCollisionWorld::PrintForceInfo(const char * n, bool detailed)
{
#ifndef DISABLE_SCREEN_LOG
	if (getNumMultibodies() != 1)
	{
		std::cout << "[error] cCollisionWorld::PrintMultibodyInfo only permit 1 multibody = " << getNumMultibodies() << std::endl;
		exit(0);
	}
	std::cout << "--------------" << n << "--------------" << std::endl;
	btMultiBody * cur_body = m_multiBodies[0];
	// output root link
	{
		std::cout << "link root " << "applied force = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseForce()).transpose() << std::endl;
		std::cout << "link root " << "applied torque = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseTorque()).transpose() << std::endl;
		std::cout << "link root " << "con force = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseConstraintedForce()).transpose() << std::endl;
		std::cout << "link root " << "con torque = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseConstraintedTorque()).transpose() << std::endl;
	}
	for (int i = 0; i < cur_body->getNumLinks(); i++)
	{
		std::cout << "link " << i << " con force = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedConstraintForce).transpose() << std::endl;
		std::cout << "link " << i << " con torque = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedConstraintTorque).transpose() << std::endl;
		std::cout << "link " << i << " applied force = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedForce).transpose() << std::endl;
		std::cout << "link " << i << " applied torque = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedTorque).transpose() << std::endl;
	}
#endif // DISABLE_SCREEN_LOG

#ifndef  DISABLE_FILE_LOG
	if (getNumMultibodies() != 1)
	{
		std::cout << "[error] cCollisionWorld::PrintMultibodyInfo only permit 1 multibody = " << getNumMultibodies() << std::endl;
		exit(0);
	}
	mWorldLog << "--------------frame " << mCurFrame << " " << n << "--------------" << std::endl;
	btMultiBody * cur_body = m_multiBodies[0];

	// output root link
	{
		mWorldLog << "link root " << "applied force = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseForce()).transpose() << std::endl;
		mWorldLog << "link root " << "applied torque = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseTorque()).transpose() << std::endl;
		mWorldLog << "link root " << "con force = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseConstraintedForce()).transpose() << std::endl;
		mWorldLog << "link root " << "con torque = " << btBulletUtil::btVectorTotVector0(cur_body->getBaseConstraintedTorque()).transpose() << std::endl;
	}
	for (int i = 0; i < cur_body->getNumLinks(); i++)
	{
		mWorldLog << "link " << i << " applied force = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedForce).transpose() << std::endl;
		mWorldLog << "link " << i << " applied torque = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedTorque).transpose() << std::endl;
		mWorldLog << "link " << i << " con force = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedConstraintForce).transpose() << std::endl;
		mWorldLog << "link " << i << " con torque = " << btBulletUtil::btVectorTotVector0(cur_body->getLink(i).m_appliedConstraintTorque).transpose() << std::endl;
	}
#endif // ! DISABLE_FILE_LOG

}

void cCollisionWorld::PrintMotionInfo(const char * n/* = nullptr*/, bool detailed)
{
#ifdef DISABLE_SCREEN_LOG
#else
	// only permit one multibody for test case
	if (getNumMultibodies() != 1)
	{
		std::cout << "[error] cCollisionWorld::PrintMultibodyInfo only permit 1 multibody = " << getNumMultibodies() << std::endl;
		exit(0);
	}

	if (n) std::cout << "--------------" << n << "--------------" << std::endl;
	btMultiBody * cur_body = m_multiBodies[0];
	Eigen::VectorXd delta_v = btBulletUtil::btArrayToEigenArray(cur_body->getDeltaV());
	Eigen::VectorXd real_buf = btBulletUtil::btArrayToEigenArray(cur_body->getRealBuf());

	if (detailed == false)
	{
		std::cout << "\tdelta v = " << delta_v.norm() << std::endl;
		std::cout << "\treal buf = " << real_buf.norm() << std::endl;
	}
	else
	{
		std::cout << "\tdelta v = " << delta_v.transpose() << std::endl;;
		std::cout << "\treal buf = " << real_buf.transpose() << std::endl;;
	}
#endif // DISABLE_SCREEN_LOG

#ifndef  DISABLE_FILE_LOG
	// only permit one multibody for test case
	if (getNumMultibodies() != 1)
	{
		std::cout << "[error] cCollisionWorld::PrintMultibodyInfo only permit 1 multibody = " << getNumMultibodies() << std::endl;
		exit(0);
	}
	
	if (n) mWorldLog << "--------------frame " << mCurFrame <<" "<< n << "--------------" << std::endl;
	btMultiBody * cur_body = m_multiBodies[0];
	Eigen::VectorXd delta_v = btBulletUtil::btArrayToEigenArray(cur_body->getDeltaV());
	Eigen::VectorXd real_buf = btBulletUtil::btArrayToEigenArray(cur_body->getRealBuf());

	if (detailed == false)
	{
		mWorldLog << "\tdelta v = " << delta_v.norm() << std::endl;
		mWorldLog << "\treal buf = " << real_buf.norm() << std::endl;
	}
	else
	{
		mWorldLog << "\tdelta v = " << delta_v.transpose() << std::endl;;
		mWorldLog << "\treal buf = " << real_buf.transpose() << std::endl;;
	}
#endif

}

void cCollisionWorld::SaveVelBuf()
{
	if (getNumMultibodies() != 1)
	{
		std::cout << "[error] cCollisionWorld::SaveVelBuf only permit 1 multibody = " << getNumMultibodies() << std::endl;
		exit(0);
	}

	btMultiBody * cur_body = m_multiBodies[0];
	mIntermediateVelBuffer = cur_body->getRealBuf();
}

void cCollisionWorld::ReleaseVelBuf()
{
	if (getNumMultibodies() != 1)
	{
		std::cout << "[error] cCollisionWorld::ReleaseVelBuf only permit 1 multibody = " << getNumMultibodies() << std::endl;
		exit(0);
	}
	btMultiBody * cur_body = m_multiBodies[0];
	cur_body->setRealBuf(mIntermediateVelBuffer);
}

void cCollisionWorld::ClearDeltaVelBuf()
{
	btMultiBody * cur_body = m_multiBodies[0];
	cur_body->clearDeltaV();
}
//
//void cCollisionWorld::integrateTransforms(btScalar timeStep)
//{
//	btDiscreteDynamicsWorld::integrateTransforms(timeStep);
//
//	{
//		BT_PROFILE("btMultiBody stepPositions");
//		//integrate and update the Featherstone hierarchies
//
//		for (int b = 0; b < m_multiBodies.size(); b++)
//		{
//			btMultiBody* bod = m_multiBodies[b];
//			bool isSleeping = false;
//			if (bod->getBaseCollider() && bod->getBaseCollider()->getActivationState() == ISLAND_SLEEPING)
//			{
//				isSleeping = true;
//			}
//			for (int b = 0; b < bod->getNumLinks(); b++)
//			{
//				if (bod->getLink(b).m_collider && bod->getLink(b).m_collider->getActivationState() == ISLAND_SLEEPING)
//					isSleeping = true;
//			}
//
//			if (!isSleeping)
//			{
//				int nLinks = bod->getNumLinks();
//
//				///base + num m_links
//
//				{
//					if (!bod->isPosUpdated())
//						bod->stepPositionsMultiDof(timeStep);
//					else
//					{
//						btScalar* pRealBuf = const_cast<btScalar*>(bod->getVelocityVector());
//						pRealBuf += 6 + bod->getNumDofs() + bod->getNumDofs() * bod->getNumDofs();
//
//						bod->stepPositionsMultiDof(1, 0, pRealBuf);
//						bod->setPosUpdated(false);
//					}
//				}
//
//				m_scratch_world_to_local.resize(nLinks + 1);
//				m_scratch_local_origin.resize(nLinks + 1);
//
//				bod->updateCollisionObjectWorldTransforms(m_scratch_world_to_local, m_scratch_local_origin);
//			}
//			else
//			{
//				bod->clearVelocities();
//			}
//		}
//	}
//}
