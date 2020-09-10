#pragma once
#include "../LCPSolverBase.h"

class cODEDantzigLCPSolver : public cLCPSolverBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	cODEDantzigLCPSolver();
	virtual ~cODEDantzigLCPSolver();
	void SetInfo(int num_of_friciton_dir, double mu, int num_of_contact, int num_of_joint_limis, bool enable_lcp);

	virtual int Solve(int num_of_vars, const tMatrixXd& A, const tVectorXd& b, tVectorXd& x);

protected:
	void transfer_question_to_rtql8(const tMatrixXd& A, const tVectorXd& b, tMatrixXd& A_rtql8, tVectorXd& b_rtql8);
	void transfer_question_from_rtql8_to_ode(const tMatrixXd& _A, const tVectorXd& _b, tMatrixXd& _AOut, tVectorXd& _bOut, int _numDir, int _numContacts);
	void transfer_sol_from_ode_to_rtql8(const tVectorXd& _x, tVectorXd& _xOut, int _numDir, int _numContacts);
	void transfer_sol_from_rtql8(const tVectorXd& x_rtql8, tVectorXd& x_self);
	void VerifySolution(const tMatrixXd& A, const tVectorXd& b, const tVectorXd& x);
	int GetSizeOfSolution();
	int GetSizeOfSolutionFriction();
	int GetSizeOfSolutionNonFriction();

	int mNumOfFrictionDir;
	bool mEnableFrictionLCP;
	double mu;
	int mNumOfContact;
	int mNumOfJointLimits;
	bool mIsInitialized;
};