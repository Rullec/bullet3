#include "btGenFBFEnergyTerm.h"
#include <iostream>
btGenFrameByFrameEnergyTerm::btGenFrameByFrameEnergyTerm(
    int size_of_solution_vector)
    : mSolutionSize(size_of_solution_vector)
{
    mAList.resize(0);
    mbList.resize(0);

    // mA.resize(0, 0);
    // mb.resize(0);
    mStartPosList.clear();
    mNameList.clear();
    mCoefList.clear();
    mRows = 0;
}
int btGenFrameByFrameEnergyTerm::GetNumOfEnergy() const
{
    std::cout << "unsupported API GetNumOfEnergy\n";
    exit(0);
    return 0;
}

/**
 * Energy term Ax + b
 */
void btGenFrameByFrameEnergyTerm::AddEnergy(const tMatrixXd &Jacobian,
                                            const tVectorXd &residual,
                                            double coeff, int st_pos,
                                            const std::string &name)
{
    mAList.push_back(Jacobian);
    mbList.push_back(residual);
    mNameList.push_back(name);
    mCoefList.push_back(coeff);
    mStartPosList.push_back(st_pos);
    mRows += Jacobian.rows();
}
void btGenFrameByFrameEnergyTerm::GetEnergyTerm(tMatrixXd &A, tVectorXd &b)
{
    A.noalias() = tMatrixXd::Zero(mRows, mSolutionSize);
    b.noalias() = tVectorXd::Zero(mRows);
    int cur_row = 0;
    for (int id = 0; id < mStartPosList.size(); id++)
    {
        int st_pos = mStartPosList[id];
        const tMatrixXd &sub_A = mAList[id];
        const tVectorXd &sub_b = mbList[id];
        int rows = sub_A.rows();
        double coef = mCoefList[id];
        A.block(cur_row, st_pos, rows, sub_A.cols()).noalias() = sub_A * coef;
        b.segment(cur_row, rows).noalias() = sub_b * coef;
        cur_row += rows;
    }
}

void btGenFrameByFrameEnergyTerm::CheckEnergyValue(const tVectorXd &sol)
{
    // std::cout << "check energy term value begin\n";
    for (int id = 0; id < mAList.size(); id++)
    {
        double coef = mCoefList[id];
        const tMatrixXd &sub_A = mAList[id];
        const tVectorXd &sub_b = mbList[id];
        tVectorXd res = (sub_A * sol + sub_b) * coef;
        double energy = res.norm();
        std::cout << "[energy] FBF " << mNameList[id] << " energy is " << energy
                  << ", coef = " << coef << std::endl;
    }
}

