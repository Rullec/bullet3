#pragma once
#include "btMultiBodyDynamicsWorld.h"
#include <fstream>

class cCollisionWorld : public btMultiBodyDynamicsWorld
{
public:
	cCollisionWorld(btDispatcher* dispatcher, btBroadphaseInterface* pairCache, btMultiBodyConstraintSolver* constraintSolver, btCollisionConfiguration* collisionConfiguration);

	~cCollisionWorld();

	virtual int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) override final;

	virtual void internalSingleStepSimulation(btScalar timeStep) override final;

	virtual void solveConstraints(btContactSolverInfo& solverInfo) override final;
	//void integrateTransforms(btScalar timeStep) override final;

protected:
	int mCurFrame;
	std::ofstream mWorldLog;

	void PrintForceInfo(const char * n = nullptr, bool detailed = false);
	void PrintMotionInfo(const char * n = nullptr, bool detailed = false);
	void SaveVelBuf();
	void ReleaseVelBuf();
	void ClearDeltaVelBuf();
	btAlignedObjectArray<btScalar> mIntermediateVelBuffer;

	// divide solveConstraint to multi funcs
	void solveCons1_ConstraintSort(btContactSolverInfo& solverInfo);	//
	void solveCons2_ApplyActiveForce(btContactSolverInfo& solverInfo);
	void solveCons3_solveCons(btContactSolverInfo& solverInfo);
	void solveCons4_finalize(btContactSolverInfo& solverInfo);
public:
	// save constrained force and torques
	btVector3 base_cons_force, base_cons_torque;
};