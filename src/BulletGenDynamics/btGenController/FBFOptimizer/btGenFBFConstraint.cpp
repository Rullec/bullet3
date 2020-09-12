// #include "IDConstraints.hpp"
#include "btGenFBFConstraint.h"
#include <iostream>
// #include "../Utils/Printer.h"

btGenFrameByFrameConstraint::btGenFrameByFrameConstraint(int size)
    : mSolutionSize(size)
{
    mNumOfEqConstraint = 0;
    mEqConstraintStPos.clear();
    mEqConstraintJacobian.clear();
    mEqConstraintResidual.clear();

    mNumOfIneqConstraint = 0;
    mIneqConstraintStPos.clear();
    mIneqConstraintJacobian.clear();
    mIneqConstraintResidual.clear();
}

int btGenFrameByFrameConstraint::GetNumOfEqCon() const
{
    return mNumOfEqConstraint;
}
int btGenFrameByFrameConstraint::GetNumOfIneqCon() const
{
    return mNumOfIneqConstraint;
}

/**
 * jac * x + residual = 0
 */
void btGenFrameByFrameConstraint::AddEqCon(const tMatrixXd &jac,
                                           const tVectorXd &residual,
                                           int st_pos)
{
    Validate(jac, residual, st_pos);

    int constraint_size = jac.rows();
    mNumOfEqConstraint += constraint_size;
    // std::cout <<"add eq " << constraint_size << std::endl;
    mEqConstraintStPos.push_back(st_pos);
    mEqConstraintJacobian.push_back(jac);
    mEqConstraintResidual.push_back(residual);
}

/**
 * \brief                   Add equivalent inequality constraints by relaxing
 * equality constraints
 */
void btGenFrameByFrameConstraint::AddEquivalentEqCon(const tMatrixXd &jac,
                                                     const tVectorXd &residual,
                                                     int st_pos,
                                                     const double tolerance)
{
    assert(tolerance >= 0);
    // jac * x = residual

    // to 1. jac * x >= residual - tol
    AddIneqCon(jac, residual - tolerance * tVectorXd::Ones(residual.size()),
               st_pos);

    // to 2. -jac * x >= -residual - tol
    AddIneqCon(-jac, -residual - tolerance * tVectorXd::Ones(residual.size()),
               st_pos);
}

// Note that jac * x >= residual
void btGenFrameByFrameConstraint::AddIneqCon(const tMatrixXd &jac,
                                             const tVectorXd &residual,
                                             int st_pos)
{
    Validate(jac, residual, st_pos);

    int constraint_size = jac.rows();
    mNumOfIneqConstraint += constraint_size;
    // std::cout <<"add ineq " << constraint_size << std::endl;
    mIneqConstraintStPos.push_back(st_pos);
    mIneqConstraintJacobian.push_back(jac);
    mIneqConstraintResidual.push_back(residual);
}

void btGenFrameByFrameConstraint::Validate(const tMatrixXd &jac,
                                           const tVectorXd &res, int st)
{
    int jac_size = jac.rows(), res_size = res.size();
    if (jac_size != res_size)
    {
        std::cout
            << "btGenFrameByFrameConstraint::Validate: the size of jacobian %d "
               "!= the size of residual %d\n";
        exit(1);
    }

    if (st < 0 || (jac.cols() + st) > mSolutionSize)
    {
        std::cout << "btGenFrameByFrameConstraint::Validate: constraint start "
                     "pos %d is illegal, "
                     "constraint length %d, solution size %d\n";
        exit(1);
    }
}

void btGenFrameByFrameConstraint::GetEqJacobianAndResidual(tMatrixXd &Jac_total,
                                                           tVectorXd &res_total)
{
    // std::cout <<"equality constraint num = " << mNumOfEqConstraint << ", cols
    // = "<< mSolutionSize << std::endl;
    Jac_total.resize(mNumOfEqConstraint, mSolutionSize);
    Jac_total.setZero();
    res_total.resize(mNumOfEqConstraint);
    res_total.setZero();

    int cur_id = 0;
    for (int i = 0; i < mEqConstraintStPos.size(); i++)
    {
        int st_pos = mEqConstraintStPos[i];
        // std::cout <<"ineq " << i <<" st " << st_pos << std::endl;
        const tMatrixXd &jac = mEqConstraintJacobian[i];
        const tVectorXd &res = mEqConstraintResidual[i];

        Jac_total.block(cur_id, st_pos, jac.rows(), jac.cols()).noalias() = jac;
        res_total.segment(cur_id, res.size()).noalias() = res;
        cur_id += jac.rows();
    }

    // if (Jac_total.rows() == 0)
    // {
    // 	Jac_total.resize(1, mSolutionSize);
    // 	Jac_total.setZero();
    // 	res_total.resize(1);
    // 	res_total.setZero();
    // }
}

void btGenFrameByFrameConstraint::GetIneqJacobianAndResidual(
    tMatrixXd &Jac_total, tVectorXd &Res_total)
{
    // std::cout <<"inequality constraint num = " << mNumOfIneqConstraint << ",
    // cols = "<< mSolutionSize << std::endl;
    Jac_total.resize(mNumOfIneqConstraint, mSolutionSize);
    Jac_total.setZero();
    Res_total.resize(mNumOfIneqConstraint);
    Res_total.setZero();

    int cur_id = 0;
    for (int i = 0; i < mIneqConstraintStPos.size(); i++)
    {
        int st_pos = mIneqConstraintStPos[i];
        // std::cout <<"ineq " << i <<" st " << st_pos << std::endl;
        const tMatrixXd &jac = mIneqConstraintJacobian[i];
        const tVectorXd &res = mIneqConstraintResidual[i];

        Jac_total.block(cur_id, st_pos, jac.rows(), jac.cols()).noalias() = jac;
        Res_total.segment(cur_id, res.size()).noalias() = res;
        cur_id += jac.rows();
    }
}