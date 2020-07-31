#include "odeDantzigLCPSolver.h"
#include "Lemke.h"
#include "lcp.h"
#include "misc.h"
#include <iostream>
#include <cstdio>

// void ExchangeIdx(tMatrixXd& mat, int i, int j)
// {
// 	// std::cout << "raw mat size = " << mat.rows() << " " << mat.cols() << std::endl;
// 	// std::cout << "i j = " << i << " " << j << std::endl;
// 	if (i == j) return;

// 	tVectorXd buf;

// 	{
// 		buf.noalias() = mat.row(i).transpose();
// 		mat.row(i).noalias() = mat.row(j);
// 		mat.row(j).noalias() = buf.transpose();
// 	}

// 	{
// 		buf.noalias() = mat.col(i);
// 		mat.col(i).noalias() = mat.col(j);
// 		mat.col(j).noalias() = buf;
// 	}
// 	// std::cout << "Result = \n"
// 	// 		  << mat << std::endl;
// 	// exit(0);
// }

// void ExchangeIdx(tVectorXd& vec, int i, int j)
// {
// 	if (i == j) return;
// 	double tmp = vec[i];
// 	vec[i] = vec[j];
// 	vec[j] = tmp;
// }

cODEDantzigLCPSolver::cODEDantzigLCPSolver() : cLCPSolverBase(eLCPSolverType::ODEDantzig), mNumOfFrictionDir(0), mu(0), mNumOfContact(0), mNumOfJointLimits(0), mIsInitialized(false)
{
	// int size = 100;
	// tMatrixXd M = tMatrixXd::Random(size, size);
	// tVectorXd n = tVectorXd::Random(size);
	// for (int i = 0; i < size; i++)
	// {
	// 	int a = std::rand() % size,
	// 		b = std::rand() % size;
	// 	tMatrixXd raw_M = M;
	// 	tVectorXd raw_n = n;
	// 	ExchangeIdx(M, a, b);
	// 	ExchangeIdx(M, a, b);
	// 	ExchangeIdx(n, a, b);
	// 	ExchangeIdx(n, a, b);
	// 	std::cout << "M_diff = " << (raw_M - M).norm() << std::endl;
	// 	std::cout << "n_diff = " << (raw_n - n).norm() << std::endl;
	// }
	// exit(1);
}

cODEDantzigLCPSolver::~cODEDantzigLCPSolver()
{
}

void cODEDantzigLCPSolver::SetInfo(int num_of_friciton_dir, double mu_, int num_of_contact, int num_of_joint_limis)
{
	mNumOfFrictionDir = num_of_friciton_dir;
	mu = mu_;
	mNumOfContact = num_of_contact;
	mNumOfJointLimits = num_of_joint_limis;
	mIsInitialized = true;
	if (mNumOfFrictionDir != 4)
	{
		std::cout << "[error] cODEDantzigLCPSolver::SetInfo friction dir != 4\n";
		exit(1);
	}
}

int cODEDantzigLCPSolver::Solve(int num_of_vars, const tMatrixXd& M, const tVectorXd& n, tVectorXd& solution)
{
	if (mIsInitialized == false)
	{
		std::cout << "[error] Please initialize the ode dantzig sover before solving\n";
		exit(1);
	}

	// 1. verify the shape of problem
	int ideal_size = GetSizeOfSolution();
	if (num_of_vars != ideal_size)
	{
		std::cout << "[error] cODEDantzigLCPSolver size inconsisitent with " << num_of_vars << " != " << ideal_size << std::endl;
		exit(1);
	}

	if (ideal_size == 0) return 0;
	// 2. transfer from mine to rtql8
	tMatrixXd A_rtql8;
	tVectorXd b_rtql8;
	// std::cout << "M=\n"
	// 		  << M << std::endl;
	// std::cout << "n="
	// 		  << n.transpose() << std::endl;
	transfer_question_to_rtql8(M, n, A_rtql8, b_rtql8);
	// std::cout << "A rtql=\n"
	// 		  << A_rtql8 << std::endl;
	// std::cout << "b rtql="
	// 		  << b_rtql8.transpose() << std::endl;
	// 3. transfer from rtql8 to ODE, then solve it
	{
		assert(mNumOfFrictionDir >= 4);
		tMatrixXd AODE;
		tVectorXd bODE;
		transfer_question_from_rtql8_to_ode(A_rtql8, b_rtql8, AODE, bODE, mNumOfFrictionDir, mNumOfContact);
		// std::cout << "A ode=\n"
		// 		  << AODE << std::endl;
		// std::cout << "b ode="
		// 		  << bODE.transpose() << std::endl;
		double *A, *b, *x, *w, *lo, *hi;
		int n = AODE.rows();

		int nSkip = dPAD(n);

		A = new double[n * nSkip];
		b = new double[n];
		x = new double[n];
		w = new double[n];
		lo = new double[n];
		hi = new double[n];
		int* findex = new int[n];

		memset(A, 0, n * nSkip * sizeof(double));
		for (int i = 0; i < n; ++i)
		{
			for (int j = 0; j < n; ++j)
			{
				A[i * nSkip + j] = AODE(i, j);
			}
		}
		for (int i = 0; i < n; ++i)
		{
			b[i] = -bODE[i];
			x[i] = w[i] = lo[i] = 0;
			hi[i] = dInfinity;
			findex[i] = -1;
		}
		for (int i = 0; i < mNumOfContact; ++i)
		{
			findex[mNumOfContact + i * 2 + 0] = i;
			findex[mNumOfContact + i * 2 + 1] = i;

			lo[mNumOfContact + i * 2 + 0] = -mu;
			lo[mNumOfContact + i * 2 + 1] = -mu;

			hi[mNumOfContact + i * 2 + 0] = mu;
			hi[mNumOfContact + i * 2 + 1] = mu;
		}
		//		dClearUpperTriangle (A,n);
		dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);
		/*
                for (int i = 0; i < n; i++) {
                    if (w[i] < 0.0 && abs(x[i] - hi[i]) > 0.000001)
                        cout << "w[" << i << "] is negative, but x is " << x[i] << endl;
                    else if (w[i] > 0.0 && abs(x[i] - lo[i]) > 0.000001)
                        cout << "w[" << i << "] is positive, but x is " << x[i] << " lo is " <<  lo[i] << endl;
                    else if (abs(w[i]) < 0.000001 && (x[i] > hi[i] || x[i] < lo[i]))
                        cout << "w[i] " << i << " is zero, but x is " << x[i] << endl;
                }
                */
		VectorXd xODE = VectorXd::Zero(n);
		for (int i = 0; i < n; ++i)
		{
			xODE[i] = x[i];
		}

		tVectorXd x_rtql8(GetSizeOfSolution());
		transfer_sol_from_ode_to_rtql8(xODE, x_rtql8, mNumOfFrictionDir, mNumOfContact);
		transfer_sol_from_rtql8(x_rtql8, solution);
		//		checkIfSolution(reducedA, reducedb, _x);

		delete[] A;
		delete[] b;
		delete[] x;
		delete[] w;
		delete[] lo;
		delete[] hi;
		delete[] findex;
	}
	mIsInitialized = false;
	VerifySolution(M, n, solution);
	// std::cout << "after verified\n";
	// exit(1);
	return 0;
}

int cODEDantzigLCPSolver::GetSizeOfSolution()
{
	return mNumOfContact * (mNumOfFrictionDir + 2) + mNumOfJointLimits;
}

void cODEDantzigLCPSolver::transfer_question_from_rtql8_to_ode(const tMatrixXd& _A, const tVectorXd& _b, tMatrixXd& _AOut, tVectorXd& _bOut, int _numDir, int _numContacts)
{
	int numOtherConstrs = _A.rows() - _numContacts * (2 + _numDir);
	int n = _numContacts * 3 + numOtherConstrs;
	tMatrixXd AIntermediate = tMatrixXd::Zero(n, _A.cols());
	_AOut = tMatrixXd::Zero(n, n);
	_bOut = tVectorXd::Zero(n);
	int offset = _numDir / 4;
	for (int i = 0; i < _numContacts; ++i)
	{
		AIntermediate.row(i) = _A.row(i);
		_bOut[i] = _b[i];

		AIntermediate.row(_numContacts + i * 2 + 0) = _A.row(_numContacts + i * _numDir + 0);
		AIntermediate.row(_numContacts + i * 2 + 1) = _A.row(_numContacts + i * _numDir + offset);
		_bOut[_numContacts + i * 2 + 0] = _b[_numContacts + i * _numDir + 0];
		_bOut[_numContacts + i * 2 + 1] = _b[_numContacts + i * _numDir + offset];
	}
	for (int i = 0; i < numOtherConstrs; i++)
	{
		AIntermediate.row(_numContacts * 3 + i) = _A.row(_numContacts * (_numDir + 2) + i);
		_bOut[_numContacts * 3 + i] = _b[_numContacts * (_numDir + 2) + i];
	}
	for (int i = 0; i < _numContacts; ++i)
	{
		_AOut.col(i) = AIntermediate.col(i);
		_AOut.col(_numContacts + i * 2 + 0) = AIntermediate.col(_numContacts + i * _numDir + 0);
		_AOut.col(_numContacts + i * 2 + 1) = AIntermediate.col(_numContacts + i * _numDir + offset);
	}
	for (int i = 0; i < numOtherConstrs; i++)
		_AOut.col(_numContacts * 3 + i) = AIntermediate.col(_numContacts * (_numDir + 2) + i);
}

void cODEDantzigLCPSolver::transfer_sol_from_ode_to_rtql8(const tVectorXd& _x, tVectorXd& _xOut, int _numDir, int _numContacts)
{
	int numOtherConstrs = _x.size() - _numContacts * 3;
	_xOut = tVectorXd::Zero(_numContacts * (2 + _numDir) + numOtherConstrs);

	_xOut.head(_numContacts) = _x.head(_numContacts);

	int offset = _numDir / 4;
	for (int i = 0; i < _numContacts; ++i)
	{
		_xOut[_numContacts + i * _numDir + 0] = _x[_numContacts + i * 2 + 0];
		_xOut[_numContacts + i * _numDir + offset] = _x[_numContacts + i * 2 + 1];
	}
	for (int i = 0; i < numOtherConstrs; i++)
		_xOut[_numContacts * (2 + _numDir) + i] = _x[_numContacts * 3 + i];
}

int GetNewId(int old_id, int num_of_contact, int num_friction)
{
	int single_size = 2 + num_friction;
	int total_size = single_size * num_of_contact;
	int group_id = static_cast<int>(std::floor(old_id / single_size));
	if (old_id >= total_size)
	{
		// other constraints
		// std::cout << "old id " << old_id << " total size = " << total_size << std::endl;
		// exit(1);
		return old_id;
	}
	if (old_id % single_size == 0)
	{
		int new_id = old_id / single_size;
		// std::cout << "old id = " << old_id << ", is normal contact, to " << new_id << std::endl;
		// ;
		return new_id;
	}

	else if (old_id % single_size == (single_size - 1))
	{
		// lowest
		int new_id = num_of_contact * (num_friction + 1) + group_id;
		// std::cout << "old id = " << old_id << ", is lambda contact, to " << new_id << std::endl;
		// ;
		return new_id;
	}
	else
	{
		//tangent
		int new_id = num_of_contact + num_friction * group_id + old_id % single_size - 1;
		// std::cout << "old id = " << old_id << ", is tangent contact, to " << new_id << std::endl;
		// ;
		return new_id;
	}
};

int GetOldId(int new_id, int num_of_contact, int num_friction)
{
	int single_size = 2 + num_friction;
	int normal_last_id = num_of_contact;
	int tangent_last_id = num_of_contact + num_friction * num_of_contact;
	int total_size = single_size * num_of_contact;
	if (new_id >= total_size)
	{
		// std::cout << "new_id " << new_id << " total size = " << total_size << std::endl;
		return new_id;
		// exit(1);
	}
	if (new_id < normal_last_id)
	{
		int old_id = single_size * new_id;
		// std::cout << "new id = " << old_id << ", is normal contact, to " << old_id << std::endl;
		return old_id;
	}

	else if (new_id < tangent_last_id)
	{
		// tangent
		int tangent_id = new_id - normal_last_id;
		int ground_id = std::floor(tangent_id / num_friction);
		int left_id = tangent_id % num_friction;
		int old_id = ground_id * single_size + 1 + left_id;
		// std::cout << "new id = " << new_id << ", is lambda contact, to " << old_id << std::endl;
		// ;
		return old_id;
	}
	else
	{
		// lambda

		int old_id = (new_id - tangent_last_id) * single_size + single_size - 1;
		// std::cout << "new id = " << old_id << ", is tangent contact, to " << old_id << std::endl;
		// ;
		return old_id;
	}
};
// TODO: optimize this part
void cODEDantzigLCPSolver::transfer_question_to_rtql8(const tMatrixXd& A, const tVectorXd& b, tMatrixXd& A_rtql8, tVectorXd& b_rtql8)
{
	// rearrange the layout from ours to rtql8's
	int size = GetSizeOfSolution();
	if (A.rows() != size || A.cols() != size)
	{
		std::cout << "[error] A size " << A.rows() << " != " << size << std::endl;
		exit(1);
	}

	A_rtql8 = A;
	b_rtql8 = b;

	// input old our id, output rtql8 id
	int* new_id_array = new int[size];
	for (int i = 0; i < size; i++)
	{
		new_id_array[i] = GetNewId(i, mNumOfContact, mNumOfFrictionDir);
	}

	for (int row = 0; row < size; row++)
	{
		int new_row = new_id_array[row];
		for (int col = 0; col < size; col++)
		{
			int new_col = new_id_array[col];
			A_rtql8(new_row, new_col) = A(row, col);
		}
		b_rtql8[new_row] = b[row];
	}
	// std::cout << "A rtql8 = \n"
	// 		  << A_rtql8 << std::endl;
	// std::cout << "b rtql8 = \n"
	// 		  << b_rtql8.transpose() << std::endl;
	// exit(1);
}

// TODO: optimize it
void cODEDantzigLCPSolver::transfer_sol_from_rtql8(const tVectorXd& x_rtql8, tVectorXd& x_self)
{
	// rearrange the layout from ours to rtql8's
	int size = GetSizeOfSolution();
	if (x_rtql8.rows() != size)
	{
		std::cout << "[error] x_rtql8 size " << x_rtql8.rows() << " != " << size << std::endl;
		exit(1);
	}

	x_self.noalias() = x_rtql8;
	for (int id = 0; id < size; id++)
	{
		x_self[GetOldId(id, mNumOfContact, mNumOfFrictionDir)] = x_rtql8[id];
	}
}

void cODEDantzigLCPSolver::VerifySolution(const tMatrixXd& A, const tVectorXd& b, const tVectorXd& x)
{
	// std::cout << "A = \n"
	// 		  << A << std::endl;
	// std::cout << "b = " << b.transpose() << std::endl;
	// std::cout << "x = " << x.transpose() << std::endl;
	// tVectorXd perp = A * x + b;
	// if(per)
	// std::cout << "[Dantzig] perp = " << perp.transpose() << std::endl;
	// exit(1);
}