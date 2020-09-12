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

#include "NativeLemkeLCPSolver.h"
#include "Lemke.h"
// #include "lcp.h"
// #include "misc.h"
#include <cstdio>
#include <iostream>

cNativeLemkeLCPSolver::cNativeLemkeLCPSolver()
    : cLCPSolverBase(eLCPSolverType::NativeLemke)
{
}

cNativeLemkeLCPSolver::~cNativeLemkeLCPSolver() {}

int cNativeLemkeLCPSolver::Solve(const tMatrixXd &_A, const tVectorXd &_b,
                                 tVectorXd &_x, double mu, int numDir,
                                 bool bUseODESolver, double *ilo, double *ihi)
{
    return lcpsolver::Lemke(_A, _b, _x);
}

void cNativeLemkeLCPSolver::transferToODEFormulation(const tMatrixXd &_A,
                                                     const tVectorXd &_b,
                                                     tMatrixXd &AOut,
                                                     tVectorXd &bOut,
                                                     int numDir)
{
    int numContacts =
        _A.rows() /
        (2 +
         numDir); // num direction 为 摩擦锥, 还有一个法向力+1, 另一个+1是乘子
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
void cNativeLemkeLCPSolver::transferSolFromODEFormulation(const tVectorXd &_x,
                                                          tVectorXd &xOut,
                                                          int numDir)
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
bool cNativeLemkeLCPSolver::checkIfSolution(const tMatrixXd &_A,
                                            const tVectorXd &_b,
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

int cNativeLemkeLCPSolver::Solve(int num_of_vars, const tMatrixXd &A,
                                 const tVectorXd &b, tVectorXd &x)
{
    // Here: never use ODE fomulation, it hasn't been well-tested
    return Solve(A, b, x, 0.0, 0, false, nullptr, nullptr);
}