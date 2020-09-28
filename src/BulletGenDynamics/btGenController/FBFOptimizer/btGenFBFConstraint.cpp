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
    mEqConstraintName.clear();

    mNumOfIneqConstraint = 0;
    mIneqConstraintStPos.clear();
    mIneqConstraintJacobian.clear();
    mIneqConstraintResidual.clear();
    mIneqConstraintName.clear();
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
                                           int st_pos, const std::string &name)
{
    Validate(jac, residual, st_pos);
    if (jac.rows() == 0)
        return;

    int constraint_size = jac.rows();
    mNumOfEqConstraint += constraint_size;
    // std::cout <<"add eq " << constraint_size << std::endl;
    mEqConstraintStPos.push_back(st_pos);
    mEqConstraintJacobian.push_back(jac);
    mEqConstraintResidual.push_back(residual);
    mEqConstraintName.push_back(name);
}

/**
 * \brief                   Add equivalent inequality constraints by relaxing
 * equality constraints
 */
void btGenFrameByFrameConstraint::AddEquivalentEqCon(const tMatrixXd &jac,
                                                     const tVectorXd &residual,
                                                     int st_pos,
                                                     const double tolerance,
                                                     const std::string &name)
{
    assert(tolerance >= 0);
    // jac * x = residual

    // to 1. jac * x >= residual - tol
    AddIneqCon(jac, residual - tolerance * tVectorXd::Ones(residual.size()),
               st_pos, "eq_" + name + "_ineq_greater");

    // to 2. -jac * x >= -residual - tol
    AddIneqCon(-jac, -residual - tolerance * tVectorXd::Ones(residual.size()),
               st_pos, "eq_" + name + "_ineq_less");
}

// Note that jac * x >= residual
void btGenFrameByFrameConstraint::AddIneqCon(const tMatrixXd &jac,
                                             const tVectorXd &residual,
                                             int st_pos,
                                             const std::string &name)
{
    Validate(jac, residual, st_pos);
    if (jac.rows() == 0)
        return;

    int constraint_size = jac.rows();
    mNumOfIneqConstraint += constraint_size;
    // std::cout <<"add ineq " << constraint_size << std::endl;
    mIneqConstraintStPos.push_back(st_pos);
    mIneqConstraintJacobian.push_back(jac);
    mIneqConstraintResidual.push_back(residual);
    mIneqConstraintName.push_back("ineq_" + name);
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
        exit(0);
    }

    if (st < 0 || (jac.cols() + st) > mSolutionSize)
    {
        std::cout << "btGenFrameByFrameConstraint::Validate: constraint start "
                     "pos %d is illegal, "
                     "constraint length %d, solution size %d\n";
        exit(0);
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

/**
 * \brief                   Check whether the constraint can be satisfied, or be
 * violated?
 */
void btGenFrameByFrameConstraint::CheckConstraint(
    const tVectorXd &total_solution)
{

    // 1. check the ineq
    // std::cout << "ineq num = " << mIneqConstraintName.size() << std::endl;
    for (int i = 0; i < mIneqConstraintName.size(); i++)
    {
        std::string name = mIneqConstraintName[i];
        int st = mIneqConstraintStPos[i];

        const tMatrixXd &jac = mIneqConstraintJacobian[i];
        const tMatrixXd &res = mIneqConstraintResidual[i];
        int size = jac.cols();
        const tVectorXd part_solution = total_solution.segment(st, size);
        const tVectorXd violate = jac * part_solution + res;

        // violate.minCoeff() >=0 should be corrent, we gave an relaxation for
        // 1e-10
        if (violate.minCoeff() < -1e-5)
        {
            std::cout << "[warn] ineq constraint " << name
                      << " solved failed, the violate value = "
                      << violate.transpose() << std::endl;

            // std::cout << "name = " << name << std::endl;
            // std::cout << "jac = \n" << jac << std::endl;
            // std::cout << "res = " << res.transpose() << std::endl;
            // std::cout << "st = " << st << std::endl;
            // std::cout << "size = " << size << std::endl;
            // std::cout << "part  = " << part_solution.transpose() << std::endl;
            // std::cout << "violate = " << violate.transpose() << std::endl;
            // std::cout << "violate min = " << violate.minCoeff() << std::endl;
            // exit(0);
        }
    }

    // 2. check the eq
    for (int i = 0; i < mEqConstraintName.size(); i++)
    {
        std::string name = mEqConstraintName[i];
        int st = mEqConstraintStPos[i];

        const tMatrixXd &jac = mEqConstraintJacobian[i];
        const tMatrixXd &res = mEqConstraintResidual[i];
        int size = jac.cols();
        const tVectorXd &part_solution = total_solution.segment(st, size);
        const tVectorXd &violate = jac * part_solution + res;
        if (violate.cwiseAbs().maxCoeff() > 1e-4)
        {
            std::cout << "[error] eq constraint " << name
                      << " solved failed, the violate value = "
                      << violate.transpose() << std::endl;
            exit(0);
        }
    }
}