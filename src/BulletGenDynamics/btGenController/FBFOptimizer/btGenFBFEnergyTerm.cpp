#include "btGenFBFEnergyTerm.h"
#include <iostream>
btGenFrameByFrameEnergyTerm::btGenFrameByFrameEnergyTerm(
    int size_of_solution_vector)
    : mSolutionSize(size_of_solution_vector)
{
    mAlist.resize(0);
    mblist.resize(0);

    // mA.resize(0, 0);
    // mb.resize(0);
    mStartPosList.clear();
    mRows = 0;
}
int btGenFrameByFrameEnergyTerm::GetNumOfEnergy() const
{
    std::cout << "unsupported API GetNumOfEnergy\n";
    exit(1);
    return 0;
}
void btGenFrameByFrameEnergyTerm::AddEnergy(const tMatrixXd &Jacobian,
                                            const tVectorXd &residual,
                                            int st_pos)
{
    mAlist.push_back(Jacobian);
    mblist.push_back(residual);
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
        const tMatrixXd &sub_A = mAlist[id];
        const tVectorXd &sub_b = mblist[id];
        int rows = sub_A.rows();

        A.block(cur_row, st_pos, rows, sub_A.cols()).noalias() = sub_A;
        b.segment(cur_row, rows).noalias() = sub_b;
        cur_row += rows;
    }
}