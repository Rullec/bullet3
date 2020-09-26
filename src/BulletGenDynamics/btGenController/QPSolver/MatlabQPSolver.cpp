#include "MatlabQPSolver.h"
#include "BulletGenDynamics/btGenController/QPSolver/SQPMatlab/libQUADASSINPROG.h"
#include "mex.h"
#include <Eigen/Sparse>
#include <limits>
#include <type_traits>

using namespace Eigen;

typedef SparseMatrix<double, ColMajor, std::make_signed<mwIndex>::type>
    MatlabSparse;

Map<MatlabSparse> matlab_to_eigen_sparse(const mxArray *mat)
{
    mxAssert(mxGetClassID(mat) == mxDOUBLE_CLASS,
             "Type of the input matrix isn't double");
    mwSize m = mxGetM(mat);
    mwSize n = mxGetN(mat);
    mwSize nz = mxGetNzmax(mat);
    /*Theoretically fails in very very large matrices*/
    mxAssert(nz <= std::numeric_limits<std::make_signed<mwIndex>::type>::max(),
             "Unsupported Data size.");
    double *pr = mxGetPr(mat);
    MatlabSparse::StorageIndex *ir =
        reinterpret_cast<MatlabSparse::StorageIndex *>(mxGetIr(mat));
    MatlabSparse::StorageIndex *jc =
        reinterpret_cast<MatlabSparse::StorageIndex *>(mxGetJc(mat));
    Map<MatlabSparse> result(m, n, nz, jc, ir, pr);
    return result;
}

mxArray *eigen_to_matlab_sparse(
    const Ref<const MatlabSparse, StandardCompressedFormat> &mat)
{
    mxArray *result =
        mxCreateSparse(mat.rows(), mat.cols(), mat.nonZeros(), mxREAL);
    const MatlabSparse::StorageIndex *ir = mat.innerIndexPtr();
    const MatlabSparse::StorageIndex *jc = mat.outerIndexPtr();
    const double *pr = mat.valuePtr();

    mwIndex *ir2 = mxGetIr(result);
    mwIndex *jc2 = mxGetJc(result);
    double *pr2 = mxGetPr(result);

    for (mwIndex i = 0; i < mat.nonZeros(); i++)
    {
        pr2[i] = pr[i];
        ir2[i] = ir[i];
    }
    for (mwIndex i = 0; i < mat.cols() + 1; i++)
    {
        jc2[i] = jc[i];
    }
    return result;
}

MatlabQPSolver::MatlabQPSolver()
{
    if (false == libQUADASSINPROGInitialize())
    {
        std::cout << "[error] qp solver init failed\n";
        exit(1);
    }
    else
    {
        std::cout << "[matlab] qp sovler init succ\n";
    }
}

void ConvertMatrixDense(mwArray &matlab_array, const tMatrixXd &eigen_mat)
{
    matlab_array =
        mwArray(eigen_mat.rows(), eigen_mat.cols(), mxDOUBLE_CLASS, mxREAL);

    if (eigen_mat.rows() > 0 && eigen_mat.cols() > 0)
        matlab_array.SetData(eigen_mat.data(),
                             eigen_mat.rows() * eigen_mat.cols());
}

bool MatlabQPSolver::Solve(int num_of_vars, tMatrixXd &H, tVectorXd &f,
                           tMatrixXd &Aeq, tVectorXd &beq, tMatrixXd &Aineq,
                           tVectorXd &bineq, int iters, tVectorXd &result)
{
    mwArray mlx, mlfval, mlexitflag, numIter, mlH, mlf, mlAineq, mlbineq, mlAeq,
        mlbeq;

    result = tVectorXd::Zero(num_of_vars);
    ConvertMatrixDense(mlx, result);

    mlfval = mwArray(1, 1, mxDOUBLE_CLASS, mxREAL);
    mlfval(1, 1) = 0;

    mlexitflag = mwArray(1, 1, mxINT64_CLASS, mxREAL);
    mlexitflag(1, 1) = 100;

    numIter = mwArray(1, 1, mxINT64_CLASS, mxREAL);
    numIter(1, 1) = iters;

    ConvertMatrixDense(mlf, f);
    ConvertMatrixDense(mlH, H);
    ConvertMatrixDense(mlAeq, Aeq);
    ConvertMatrixDense(mlbeq, beq);
    ConvertMatrixDense(mlAineq, Aineq);
    ConvertMatrixDense(mlbineq, bineq);

    int nargout = 3;
    QUADASSINPROG(nargout, mlx, mlfval, mlexitflag, mlH, mlf, mlAineq, mlbineq,
                  mlAeq, mlbeq, numIter);

    bool succ = true;
    if (1 != static_cast<int>(mlexitflag(1, 1)))
    {
        std::cout << "[error] matlabQPSOlver extflag = " << mlexitflag
                  << std::endl;
        succ &= false;
    }

    if (mlx.NumberOfElements() == num_of_vars)
    {
        for (int i = 0; i < num_of_vars; i++)
        {
            result[i] = mlx(i + 1, 1);
        }
    }
    return succ;
    // std::cout <<"result = " << x.segment(0,
    // std::min(static_cast<int>(x.size()), 100)).transpose() << std::endl;
}

// void MatlabQPSolver::SparseSolve(int num_of_vars,sparse_mat & H, tVectorXd &
// f, sparse_mat & Aeq, tVectorXd & beq, sparse_mat & Aineq, tVectorXd & bineq,
// int iters)
// {
//     mwArray mlH = mwArray::NewSparse(1, 1, H.size(), mxDOUBLE_CLASS);
//     mlH.SetData()

//     mwArray mlx, mlfval, mlexitflag, numIter, mlf, mlAineq, mlbineq, mlAeq,
//     mlbeq;

//     tVectorXd x = tVectorXd::Zero(num_of_vars);
//     ConvertMatrixDense(mlx, x);

//     mlfval = mwArray(1, 1, mxDOUBLE_CLASS, mxREAL);
//     mlfval(1, 1) = 0;

//     mlexitflag = mwArray(1, 1, mxINT64_CLASS, mxREAL);
//     mlexitflag(1, 1) = 100;

// 	numIter = mwArray(1, 1, mxINT64_CLASS, mxREAL);
// 	numIter(1, 1) = iters;

// 	ConvertMatrixDense(mlf, f);
// 	ConvertMatrixDense(mlH, H);
// 	ConvertMatrixDense(mlAeq, Aeq);
// 	ConvertMatrixDense(mlbeq, beq);
// 	ConvertMatrixDense(mlAineq, Aineq);
// 	ConvertMatrixDense(mlbineq, bineq);
//     int nargout = 3;
//     SolveQP(nargout, mlx, mlfval, mlexitflag, mlH, mlf, mlAineq, mlbineq,
//     mlAeq, mlbeq, numIter);

//     for(int i=0; i<num_of_vars; i++)
//     {
//         x[i] = mlx(i+1, 1);
//     }
//     std::cout <<"result = " << x.segment(0,
//     std::min(static_cast<int>(x.size()), 100)).transpose() << std::endl;
// }