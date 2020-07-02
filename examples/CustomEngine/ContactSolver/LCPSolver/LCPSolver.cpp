/*
 * RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jie (Jay) Tan <jtan34@cc.gatech.edu>
 *
 * Geoorgia Tech Graphics Lab
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "LCPSolver.h"
#include "Lemke.h"
#include "lcp.h"
#include "misc.h"
#include <cstdio>
#include <iostream>

cLCPSolver::cLCPSolver() {}
cLCPSolver::~cLCPSolver() {}

int cLCPSolver::Solve(const tMatrixXd &_A, const tVectorXd &_b, tVectorXd &_x,
					  double mu, int numDir, bool bUseODESolver, double *ilo,
					  double *ihi)
{
	if (!bUseODESolver)
	{
		int err = lcpsolver::Lemke(_A, _b, _x);
		// Eigen::EigenSolver<tMatrixXd> eigen_solver(_A);
		// std::cout << "A eigen value" << eigen_solver.eigenvalues().real().transpose() << std::endl;
		return err;
	}
	else
	{
		assert(numDir >= 4);
		tMatrixXd AODE;
		tVectorXd bODE;
		transferToODEFormulation(_A, _b, AODE, bODE, numDir);
		// int err = Lemke(AODE, -bODE, _x);
		double *A, *b, *x, *w, *lo, *hi;
		int n = AODE.rows();
		int nContacts = n / 3;

		int nSkip = dPAD(n);

		A = new double[n * nSkip];
		b = new double[n];
		x = new double[n];
		w = new double[n];
		// if (ilo == nullptr)
		lo = new double[n];
		// if (ihi == nullptr)
		hi = new double[n];
		int *findex = new int[n];

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
		}
		for (int i = 0; i < nContacts; ++i)
		{
			findex[i] = -1;
			findex[nContacts + i * 2 + 0] = i;
			findex[nContacts + i * 2 + 1] = i;

			lo[nContacts + i * 2 + 0] = -mu;
			lo[nContacts + i * 2 + 1] = -mu;

			hi[nContacts + i * 2 + 0] = mu;
			hi[nContacts + i * 2 + 1] = mu;
		}
		// memsetk                tVectorXd comple_cond = (A * x +
		// b).cwiseProduct(x);
		// std::cout << "comple cond = " << comple_cond.cwiseAbs().maxCoeff()
		//           << std::endl;
		// (lo, 0, sizeof(double) * n);
		//		dClearUpperTriangle (A,n);
		dSolveLCP(n, A, x, b, w, 0, lo, hi, findex);

		tVectorXd xODE = tVectorXd::Zero(n);
		for (int i = 0; i < n; ++i)
		{
			xODE[i] = x[i];
		}
		transferSolFromODEFormulation(xODE, _x, numDir);

		//		checkIfSolution(reducedA, reducedb, _x);

		delete[] A;
		delete[] b;
		delete[] x;
		delete[] w;
		delete[] lo;
		delete[] hi;
		delete[] findex;
		return 1;
	}
}

void cLCPSolver::transferToODEFormulation(const tMatrixXd &_A,
										  const tVectorXd &_b, tMatrixXd &AOut,
										  tVectorXd &bOut, int numDir)
{
	int numContacts =
		_A.rows() /
		(2 +
		 numDir);  // num direction 为 摩擦锥, 还有一个法向力+1, 另一个+1是乘子
	tMatrixXd AIntermediate = tMatrixXd::Zero(numContacts * 3, _A.cols());
	AOut = tMatrixXd::Zero(numContacts * 3, numContacts * 3);
	bOut = tVectorXd::Zero(numContacts * 3);
	int offset = numDir / 4;
	for (int i = 0; i < numContacts; ++i)
	{
		AIntermediate.row(i) = _A.row(i);
		bOut[i] = _b[i];

		AIntermediate.row(numContacts + i * 2 + 0) =
			_A.row(numContacts + i * numDir + 0);
		AIntermediate.row(numContacts + i * 2 + 1) =
			_A.row(numContacts + i * numDir + offset);
		bOut[numContacts + i * 2 + 0] = _b[numContacts + i * numDir + 0];
		bOut[numContacts + i * 2 + 1] = _b[numContacts + i * numDir + offset];
	}
	for (int i = 0; i < numContacts; ++i)
	{
		AOut.col(i) = AIntermediate.col(i).eval();
		AOut.col(numContacts + i * 2 + 0) =
			AIntermediate.col(numContacts + i * numDir + 0).eval();
		AOut.col(numContacts + i * 2 + 1) =
			AIntermediate.col(numContacts + i * numDir + offset).eval();
	}
}
void cLCPSolver::transferSolFromODEFormulation(const tVectorXd &_x,
											   tVectorXd &xOut, int numDir)
{
	int numContacts = _x.size() / 3;
	xOut = tVectorXd::Zero(numContacts * (2 + numDir));

	xOut.head(numContacts) = _x.head(numContacts);

	int offset = numDir / 4;
	for (int i = 0; i < numContacts; ++i)
	{
		xOut[numContacts + i * numDir + 0] = _x[numContacts + i * 2 + 0];
		xOut[numContacts + i * numDir + offset] = _x[numContacts + i * 2 + 1];
	}
}
bool cLCPSolver::checkIfSolution(const tMatrixXd &_A, const tVectorXd &_b,
								 const tVectorXd &_x)
{
	const double threshold = 1e-4;
	int n = _x.size();

	tVectorXd w = _A * _x + _b;
	for (int i = 0; i < n; ++i)
	{
		if (w(i) < -threshold || _x(i) < -threshold)
			return false;
		if (abs(w(i) * _x(i)) > threshold)
			return false;
	}
	return true;
}