#include "btPath.h"
#include "btBulletDynamicsCommon.h"
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define NEED_TO_SOLVE 0
#define SOLVE_FINISHED 1

#define MAX_CONTACT_NUM (100)
#define SINGLE_SIZE (6 * MAX_CONTACT_NUM)

// define the same shared memory with PATH LCP solver server.
static const double threshold = 1e-6;
struct memory
{
	int nonzero_i_idx[SINGLE_SIZE * SINGLE_SIZE];
	int nonzero_j_idx[SINGLE_SIZE * SINGLE_SIZE];
	double nonzero_entries[SINGLE_SIZE * SINGLE_SIZE];
	double b[SINGLE_SIZE];
	double x[SINGLE_SIZE];
	int num_of_vars;
	int num_of_nonzeros;
	int status;
};
static volatile struct memory* memory;
static void FormNewQuestion(const tMatrixXd& A, const tVectorXd& b);
static void VerifyAnswer(const tMatrixXd& A, const tVectorXd& b, tVectorXd& x);

cBulletPathLCPSolver::cBulletPathLCPSolver() : cLCPSolverBase(eLCPSolverType::BulletLemke)
{
	// create shared memory
	int shmid;
	// key value of shared memory
	int key = 12345;
	// shared memory create
	shmid = shmget(key, sizeof(struct memory), IPC_CREAT | 0666);
	if (-1 == shmid)
	{
		std::cout << "[error] BulletPathLCPSolver create shared memory failed: ";
		switch (errno)
		{
			case EACCES:
				std::cout << "EACCES";
				break;
			case EEXIST:
				std::cout << "EEXIST";
				break;
			case EINVAL:
				std::cout << "EINVAL";
				break;
			case ENFILE:
				std::cout << "ENFILE";
				break;
			case ENOENT:
				std::cout << "ENOENT";
				break;
			case ENOMEM:
				std::cout << "ENOMEM";
				break;
			case EPERM:
				std::cout << "EPERM";
				break;

			default:
				std::cout << " UNKNOWN error";
				break;
		}
		std::cout << std::endl;
		exit(1);
	}
	memory = (struct memory*)shmat(shmid, NULL, 0);
}

/**
 * \brief			Solve 0 <= x \perp M * x + n >=0
*/
#include "BulletGenDynamics/btGenUtil/TimeUtil.hpp"
int cBulletPathLCPSolver::Solve(int num_of_vars, const tMatrixXd& M, const tVectorXd& n, tVectorXd& x_result)
{
	while (memory->status != SOLVE_FINISHED)
		continue;

	// now we can try to put a new problem in the shared memory
	btTimeUtil::Begin("form questions");
	FormNewQuestion(M, n);
	// std::cout << "M size = " << M.rows() << " " << M.cols() << std::endl;
	btTimeUtil::End("form questions");
	// std::cout << "set up question for size " << memory->num_of_vars
	// 		  << std::endl;
	btTimeUtil::Begin("solving shared memory");
	memory->status = NEED_TO_SOLVE;
	while (memory->status != SOLVE_FINISHED)
		continue;
	btTimeUtil::End("solving shared memory");

	VerifyAnswer(M, n, x_result);
	return 0;
}

// tMatrixXd ConvertbtMatrixXd

void cBulletPathLCPSolver::Test()
{
}

void FormNewQuestion(const tMatrixXd& A, const tVectorXd& b)
{
	// // 1. prepare
	// // int num_of_vars = 10;
	// int num_of_vars = std::rand() % (MAX_CONTACT_NUM) + 1;
	// A = tMatrixXd::Random(num_of_vars, num_of_vars);
	// // A = tMatrixXd::Identity(num_of_vars, num_of_vars);
	// A = A.transpose() * A;
	// b = tVectorXd::Random(num_of_vars);

	// set up buffer
	int n_vars = A.rows();
	memory->num_of_vars = n_vars;
	memory->num_of_nonzeros = 0;

	for (int i = 0; i < n_vars; i++)
	{
		memory->x[i] = 0;
		for (int j = 0; j < n_vars; j++)
		{
			if (std::fabs(A(i, j)) > threshold)
			{
				memory->nonzero_i_idx[memory->num_of_nonzeros] = i + 1;
				memory->nonzero_j_idx[memory->num_of_nonzeros] = j + 1;
				memory->nonzero_entries[memory->num_of_nonzeros] = A(i, j);
				memory->num_of_nonzeros = memory->num_of_nonzeros + 1;
			}
		}
		memory->b[i] = b[i];
	}
}

void VerifyAnswer(const tMatrixXd& A, const tVectorXd& b, tVectorXd& x)
{
	x = tVectorXd::Zero(memory->num_of_vars);
	for (int i = 0; i < memory->num_of_vars; i++)
	{
		x[i] = memory->x[i];
		// std::cout << "x" << i << " = " << memory->x[i] << std::endl;
	}

	tVectorXd res = A * x + b;

	// std::cout << "x = " << x.transpose() << std::endl;
	double eps = 1e-4;
	tVectorXd perp = res.cwiseProduct(x);
	if (perp.cwiseAbs().maxCoeff() > eps || x.minCoeff() < -eps ||
		res.minCoeff() < -eps)
	// std::cout << "--------------------------------\n";
	// if (true)
	{
		std::cout << "[PATH] verify failed\n";
		btMathUtil::RoundZero(x, 1e-8);
		btMathUtil::RoundZero(res, 1e-8);
		std::cout << "[PATH] x = " << x.transpose() << std::endl;

		// std::cout << "[PATH] x abs max = " << x.cwiseAbs().maxCoeff() << std::endl;
		// std::cout << "[PATH] x min = " << x.minCoeff() << std::endl;
		std::cout << "[PATH] residual = " << res.transpose() << std::endl;
		// std::cout << "[PATH] residual min = " << res.minCoeff() << std::endl;
		std::cout << "[PATH] perp max = " << perp.cwiseAbs().maxCoeff() << std::endl;
		// btMathUtil::RoundZero(A.array());
		// btMathUtil::RoundZero(b.array());
		if (A.rows() <= 6)
		{
			tMatrixXd Abak = A;
			tVectorXd bbak = b;
			btMathUtil::RoundZero(Abak);
			btMathUtil::RoundZero(bbak);
			std::cout << "[PATH] M = \n"
					  << Abak << std::endl;
			std::cout << "[PATH] n = \n"
					  << bbak.transpose() << std::endl;
		}
	}
	else
		std::cout << "[PATH] LCP solved correct for size " << memory->num_of_vars
				  << std::endl;
}
