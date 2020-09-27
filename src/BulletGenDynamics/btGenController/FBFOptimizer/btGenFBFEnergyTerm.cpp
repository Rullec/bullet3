#include "btGenFBFEnergyTerm.h"
#include <fstream>
#include <iostream>
std::string energy_file = "energy.txt";
btGenFrameByFrameEnergyTerm::btGenFrameByFrameEnergyTerm()
{
    mAList.resize(0);
    mbList.resize(0);

    // mA.resize(0, 0);
    // mb.resize(0);
    mStartPosList.clear();
    mNameList.clear();
    mCoefList.clear();
    mRows = 0;
    mSolutionSize = -1;

    std::ofstream fout(energy_file);
    fout << "";
    fout.close();
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
        // std::cout << "[FBF] A sub " << mNameList[id]
        //           << " norm = " << sub_A.norm() << std::endl;
        // std::cout << "[FBF] b sub " << mNameList[id]
        //           << " norm = " << sub_b.norm() << std::endl;
    }
}

void btGenFrameByFrameEnergyTerm::CheckEnergyValue(const tVectorXd &sol)
{
    // std::cout << "check energy term value begin\n";
    std::ofstream fout(energy_file, std::ios::app);
    for (int id = 0; id < mAList.size(); id++)
    {
        double coef = mCoefList[id];
        const tMatrixXd &sub_A = mAList[id];
        const tVectorXd &sub_b = mbList[id];
        int st_pos = mStartPosList[id];
        const tVectorXd part_sol = sol.segment(st_pos, sub_A.cols());
        // std::cout << "name = " << mNameList[id] << std::endl;
        // std::cout << "A size = " << sub_A.rows() << " " << sub_A.cols()
        //           << std::endl;
        // std::cout << "b size = " << sub_b.rows() << " " << sub_b.cols()
        //           << std::endl;
        tVectorXd res = (sub_A * part_sol + sub_b) * coef;
        double energy = res.norm();

        // std::cout << "sol size = " << sol.size() << std::endl;

        std::cout << "[energy] FBF " << mNameList[id] << " energy is " << energy
                  << ", coef = " << coef << std::endl;
        fout << "[energy] FBF " << mNameList[id] << " energy is " << energy
             << ", coef = " << coef << std::endl;
    }
    fout.close();

    // if there is not contact point, test whether the solution is the same as
    // the least-square sol?
    // {
    //     const tMatrixXd &sub_A = mAList[0];
    //     const tVectorXd &sub_b = mbList[0];
    //     tVectorXd lsq_sol =
    //         -(sub_A.transpose() * sub_A).inverse() * sub_A.transpose() *
    //         sub_b;
    //     tVectorXd diff = sol - lsq_sol;
    //     std::cout << "lsq sol = " << lsq_sol.transpose() << std::endl;
    //     std::cout << "opt sol = " << sol.transpose() << std::endl;
    //     std::cout << "diff = " << diff.transpose() << std::endl;
    // }
}

void btGenFrameByFrameEnergyTerm::Reset(int sol_size)
{
    mSolutionSize = sol_size;
    mNameList.clear();
    mCoefList.clear();
    mAList.clear();
    mbList.clear();
    mStartPosList.clear();
    mRows = 0;
}