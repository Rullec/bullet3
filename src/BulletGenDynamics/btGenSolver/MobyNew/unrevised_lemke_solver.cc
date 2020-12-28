#include "unrevised_lemke_solver.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "eigen_autodiff_types.h"
#include "eigen_types.h"
#include <Eigen/LU>
bool output = false;
// #include "drake/common/autodiff.h"
// #include "drake/common/default_scalars.h"
// #include "drake/common/drake_assert.h"
// #include "drake/common/never_destroyed.h"
// #include "drake/common/text_logging.h"
// #include "drake/common/unused.h"

// using drake::log;

// namespace drake
// {
// namespace solvers
// {

namespace
{

// A linear system solver that accommodates the inability of the LU
// factorization to be AutoDiff'd (true in Eigen 3, at least). For double types,
// the faster LU factorization and solve is used. For other types, QR
// factorization and solve is used.
class LinearSolver
{
public:
    explicit LinearSolver(const tMatrixXd &m);

    tVectorXd Solve(const tVectorXd &v) const;

private:
    Eigen::ColPivHouseholderQR<tMatrixXd> qr_;
    Eigen::PartialPivLU<MatrixX<double>> lu_;
};

LinearSolver::LinearSolver(const MatrixX<double> &M)
{
    if (M.rows() > 0)
        lu_ = Eigen::PartialPivLU<MatrixX<double>>(M);
}

// tVectorXd LinearSolver::Solve(const tVectorXd &v) const
// {
//     if (v.rows() == 0)
//     {
//         assert(qr_.rows() == 0);
//         return tVectorXd(0);
//     }
//     return qr_.solve(v);
// }

tVectorXd LinearSolver::Solve(const tVectorXd &v) const
{
    if (v.rows() == 0)
    {
        assert(lu_.rows() == 0);
        return tVectorXd(0);
    }
    return lu_.solve(v);
}
} // anonymous namespace

// template <>
// void UnrevisedLemkeSolver<AutoDiffXd>::DoSolve(
//     const MathematicalProgram &, const Eigen::VectorXd &, const SolverOptions &,
//     MathematicalProgramResult *) const
// {
//     throw std::logic_error(
//         "UnrevisedLemkeSolver cannot yet be used in a "
//         "MathematicalProgram while templatized as an AutoDiff");
// }

//
// void UnrevisedLemkeSolver::DoSolve(const MathematicalProgram &prog,
//                                       const Eigen::VectorXd &initial_guess,
//                                       const SolverOptions &merged_options,
//                                       MathematicalProgramResult *result) const
// {
//     if (!prog.GetVariableScaling().empty())
//     {
//         static const logging::Warn log_once(
//             "UnrevisedLemkeSolver doesn't support the feature of variable "
//             "scaling.");
//     }

//     unused(initial_guess);
//     unused(merged_options);

//     // Solve each individual LCP, writing the result back to the decision
//     // variables through the binding and returning true iff all LCPs are
//     // feasible.
//     //
//     // If any is infeasible, returns false and does not alter the decision
//     // variables.

//     // Create a dummy variable for the number of pivots used.
//     int num_pivots = 0;
//     const auto &bindings = prog.linear_complementarity_constraints();
//     Eigen::VectorXd x_sol(prog.num_vars());
//     for (const auto &binding : bindings)
//     {
//         Eigen::VectorXd constraint_solution(binding.GetNumElements());
//         const std::shared_ptr<LinearComplementarityConstraint> constraint =
//             binding.evaluator();
//         bool solved = SolveLcpLemke(constraint->M(), constraint->q(),
//                                     &constraint_solution, &num_pivots);
//         if (!solved)
//         {
//             result->set_solution_result(SolutionResult::kUnknownError);
//             return;
//         }
//         for (int i = 0; i < binding.evaluator()->num_vars(); ++i)
//         {
//             const int variable_index =
//                 prog.FindDecisionVariableIndex(binding.variables()(i));
//             x_sol(variable_index) = constraint_solution(i);
//         }
//     }
//     result->set_optimal_cost(0.0);
//     result->set_x_val(x_sol);
//     result->set_solution_result(SolutionResult::kSolutionFound);
// }

// Utility function for copying an r-dimensional column vector v (designated by
// the indices in row_indices and col_index) from a matrix M to a target
// vector, `out`. Let M be the matrix `in` augmented with a single column of
// ones (i.e., the "covering vector"); put another way, M = | in 1 |, where "in"
// refers the n × m-dimensional matrix `in` and 1 is a column of ones (so M is
// n × (m+1)-dimensional). Let R be a r × n "row selection" matrix, constructed
// using `row_indices`. R is constructed using r = `row_indices.size()` as
// follows:
//
// Rᵢⱼ = { 1  if j = row_indices[i], ∀i ∈ 0..r-1, ∀j ∈ 0..n-1
//       { 0  otherwise
//
// Consider the following example with `in` set to the 3 × 3 identity matrix,
// `row_indices = { 2, 0}` and `col_index = 1`. This would make:
// R =  | 0 0 1 |      and R⋅M = | 0 0 1 1 |
//      | 1 0 0 |                | 1 0 0 1 |
//
// The second column (`col_index = 1`) of R⋅M is v = | 0 |
//                                                   | 0 |
//
// `row_indices` need not be in sorted order, and `out` need not be properly
// sized on entry (it will be resized as necessary). However, this method aborts
// if any element of `row_indices` is out of range, `col_index` is out of range,
// *or* if there are any duplicated elements in `row_indices`.

void UnrevisedLemkeSolver::SelectSubColumnWithCovering(
    const tMatrixXd &in, const std::vector<int> &row_indices, int col_index,
    tVectorXd *out)
{
    DRAKE_ASSERT(ValidateIndices(row_indices, in.rows()));

    const int num_rows = row_indices.size();
    out->resize(num_rows);

    // Look for the covering vector first.
    if (col_index == in.cols())
    {
        out->setOnes();
        return;
    }

    assert(0 <= col_index && col_index < in.cols());
    const auto in_column = in.col(col_index);
    for (int i = 0; i < num_rows; i++)
    {
        DRAKE_ASSERT(row_indices[i] < in_column.size());
        (*out)[i] = in_column[row_indices[i]];
    }
}

// Utility function for copying an r-dimensional column vector v (designated by
// the indices in `row_indices`) from n-dimensional vector `in` to a target
// vector, `out`. Let R be a r × n "row selection" matrix, constructed using
// r = `row_indices.size()` as follows:
//
// Rᵢⱼ = { 1  if j = row_indices[i], ∀i ∈ 0..r-1, ∀j ∈ 0..n-1
//       { 0  otherwise
//
// Consider the following example with `in` set to the vector [ 1 2 3 ]ᵀ,
// and `row_indices = { 2, 0}`. This would make:
// R =  | 0 0 1 |      and v = R⋅`in` = | 3 |
//      | 1 0 0 |                       | 1 |
//
// `row_indices` need not be in sorted order, and `out` need not be properly
// sized on entry (it will be resized as necessary). However, this method aborts
// if any element of `row_indices` is out of range  *or* if there are any
// duplicated elements in `row_indices`.

void UnrevisedLemkeSolver::SelectSubVector(const tVectorXd &in,
                                           const std::vector<int> &row_indices,
                                           tVectorXd *out)
{
    DRAKE_ASSERT(ValidateIndices(row_indices, in.size()));

    const int num_rows = row_indices.size();
    out->resize(num_rows);
    for (int i = 0; i < num_rows; i++)
    {
        DRAKE_ASSERT(row_indices[i] < in.rows());
        (*out)(i) = in(row_indices[i]);
    }
}

// Utility function for copying an r-dimensional column vector v (designated by
// the indices in `row_indices`) into n-dimensional vector `out`. Let R be the
// r × n "row selection" matrix defined in the documentation of
// SelectSubVector(). Then, the result of the operation is:
// `out` = `out` + Rᵀ⋅(`v` - R⋅`out`)
//
// Consider the following example with `v` set to the vector [ 3 1 ]ᵀ,
// `row_indices = { 2, 0}`, and `out` set to the vector [ 4 5 6 ]. This would
// make:
// R =  | 0 0 1 |      and `out` + Rᵀ⋅(`v` - R⋅`out`) = | 1 |
//      | 1 0 0 |                                       | 5 |
//                                                      | 3 |
//
// `row_indices` need not be in sorted order. This method aborts if any element
// of `row_indices` is out of the range of `out`, if `row_indices` is not the
// same size as `v`, or if there are any duplicated elements in `row_indices`.

void UnrevisedLemkeSolver::SetSubVector(const tVectorXd &v,
                                        const std::vector<int> &row_indices,
                                        tVectorXd *out)
{
    assert(row_indices.size() == static_cast<size_t>(v.size()));
    DRAKE_ASSERT(ValidateIndices(row_indices, out->size()));
    for (size_t i = 0; i < row_indices.size(); ++i)
        (*out)[row_indices[i]] = v[i];
}

// Function for checking whether a set of indices that specify a view into
// a vector is valid. Returns `true` if row_indices are unique and each element
// lies in [0, vector_size-1] and `false` otherwise.

bool UnrevisedLemkeSolver::ValidateIndices(const std::vector<int> &row_indices,
                                           int vector_size)
{
    // Don't check anything for empty vectors.
    if (row_indices.empty())
        return true;

    // Sort the vector first.
    std::vector<int> sorted_row_indices = row_indices;
    std::sort(sorted_row_indices.begin(), sorted_row_indices.end());

    // Validate the maximum and minimum elements.
    if (sorted_row_indices.back() >= vector_size)
        return false;
    if (sorted_row_indices.front() < 0)
        return false;

    // Make sure that the vector is unique.
    return std::unique(sorted_row_indices.begin(), sorted_row_indices.end()) ==
           sorted_row_indices.end();
}

// Function for checking whether a set of indices that specify a view into
// a matrix is valid. Returns `true` if each element of row_indices is unique
// lies in [0, num_rows-1] and if each element of col_indices is unique and
// lies in [0, num_cols-1]. Returns `false` otherwise.

bool UnrevisedLemkeSolver::ValidateIndices(const std::vector<int> &row_indices,
                                           const std::vector<int> &col_indices,
                                           int num_rows, int num_cols)
{
    return ValidateIndices(row_indices, num_rows) &&
           ValidateIndices(col_indices, num_cols);
}

// Utility function for copying an r × c dimensional submatrix S (designated by
// the indices in row_indices and col_indices) from a matrix M to a target
// matrix, `out`. Let M be the matrix `in` augmented with a single column of
// ones (i.e., the "covering vector"); put another way, M = | in 1 |, where "in"
// refers the n × m-dimensional matrix `in` and 1 is a column of ones (so M is
// n × (m+1)-dimensional). Let R be a r × n "row selection" matrix, constructed
// using `row_indices` and C be a (m+1) × c-dimensional "column selection"
// matrix constructed using `col_indices`. R and C are constructed using
// r = `row_indices.size()` and c = `col_indices.size()` as follows:
//
// Rᵢⱼ = { 1  if j = row_indices[i], ∀i ∈ 0..r-1, ∀j ∈ 0..n-1
//       { 0  otherwise
// Cᵢⱼ = { 1  if col_indices[j] = i, ∀i ∈ 0..m, ∀j ∈ 0..c-1
//       { 0  otherwise
//
// Consider the following example with `in` set to the 3 × 3 identity matrix,
// `row_indices = { 2, 1, 0}` and `col_indices = { 1, 2, 3 }`. This would make:
// R =  | 0 0 1 |      C = | 0 0 0 |   and  R⋅M⋅C = | 0 1 1 |
//      | 0 1 0 |          | 1 0 0 |                | 1 0 1 |
//      | 1 0 0 |          | 0 1 0 |                | 0 0 1 |
//                         | 0 0 1 |
// `row_indices` and `col_indices` need not be in sorted order, and `out` need
// not be properly sized on entry (it will be resized as necessary). However,
// this method aborts if any element of `row_indices` or `col_indices` is out
// of range *or* if there are any duplicated elements.

void UnrevisedLemkeSolver::SelectSubMatrixWithCovering(
    const tMatrixXd &in, const std::vector<int> &row_indices,
    const std::vector<int> &col_indices, tMatrixXd *out)
{
    const int num_rows = row_indices.size();
    const int num_cols = col_indices.size();
    DRAKE_ASSERT(
        ValidateIndices(row_indices, col_indices, in.rows(), in.cols() + 1));
    out->resize(num_rows, num_cols);

    for (int i = 0; i < num_rows; i++)
    {
        const auto row_in = in.row(row_indices[i]);

        // `row_out` is a "view" into `out`: any modifications to row_out are
        // reflected in `out`.
        auto row_out = out->row(i);
        for (int j = 0; j < num_cols; j++)
        {
            if (col_indices[j] < in.cols())
            {
                DRAKE_ASSERT(col_indices[j] >= 0);
                row_out(j) = row_in(col_indices[j]);
            }
            else
            {
                DRAKE_ASSERT(col_indices[j] == in.cols());
                row_out(j) = 1.0;
            }
        }
    }
}

// Determines the various index sets defined in Section 1.1 of [Dai 2018].
void UnrevisedLemkeSolver::DetermineIndexSets() const
{
    // Helper for determining index sets.
    auto DetermineIndexSetsHelper =
        [this](const std::vector<LCPVariable> &variables, bool is_z,
               std::vector<int> *variable_set,
               std::vector<int> *variable_set_prime) {
            variable_and_array_indices_.clear();
            for (int i = 0; i < static_cast<int>(variables.size()); ++i)
            {
                if (variables[i].is_z() == is_z)
                    variable_and_array_indices_.emplace_back(
                        variables[i].index(), i);
            }
            std::sort(variable_and_array_indices_.begin(),
                      variable_and_array_indices_.end());

            // Construct the set and the primed set.
            for (const auto &variable_and_array_index_pair :
                 variable_and_array_indices_)
            {
                variable_set->push_back(variable_and_array_index_pair.first);
                variable_set_prime->push_back(
                    variable_and_array_index_pair.second);
            }
        };

    // Clear all sets.
    index_sets_.alpha.clear();
    index_sets_.alpha_bar.clear();
    index_sets_.alpha_prime.clear();
    index_sets_.alpha_bar_prime.clear();
    index_sets_.beta.clear();
    index_sets_.beta_bar.clear();
    index_sets_.beta_prime.clear();
    index_sets_.beta_bar_prime.clear();

    DetermineIndexSetsHelper(indep_variables_, false, &index_sets_.alpha,
                             &index_sets_.alpha_prime);
    DetermineIndexSetsHelper(dep_variables_, false, &index_sets_.alpha_bar,
                             &index_sets_.alpha_bar_prime);
    DetermineIndexSetsHelper(dep_variables_, true, &index_sets_.beta,
                             &index_sets_.beta_prime);
    DetermineIndexSetsHelper(indep_variables_, true, &index_sets_.beta_bar,
                             &index_sets_.beta_bar_prime);
}

// Verifies that each element of the pivoting set is unique. This is an
// expensive operation and should only be executed in Debug mode.

bool UnrevisedLemkeSolver::IsEachUnique(const std::vector<LCPVariable> &vars)
{
    // Copy the set.
    std::vector<LCPVariable> vars_copy = vars;
    std::sort(vars_copy.begin(), vars_copy.end());
    return (std::unique(vars_copy.begin(), vars_copy.end()) == vars_copy.end());
}

// Performs the pivoting operation, which is described in [Dai 2018].
// `M_prime_col` can be null, if the updated column of M' (pivoted version of M)
// is not needed and the driving_index corresponds to the artificial variable.

bool UnrevisedLemkeSolver::LemkePivot(const tMatrixXd &M, const tVectorXd &q,
                                      int driving_index, double zero_tol,
                                      tVectorXd *M_prime_col,
                                      tVectorXd *q_prime) const
{
    assert(q_prime);

    const int kArtificial = M.rows();
    assert(driving_index >= 0 && driving_index <= kArtificial);

    // Verify that each member in the independent and dependent sets is unique.
    DRAKE_ASSERT(IsEachUnique(indep_variables_));
    DRAKE_ASSERT(IsEachUnique(dep_variables_));

    // If the driving index does not correspond to the artificial variable,
    // M_prime_col must be non-null.
    if (!IsArtificial(indep_variables_[driving_index]))
        assert(M_prime_col);

    // Determine the sets.
    DetermineIndexSets();

    // Note: It is feasible to do a low-rank update to the factorization below,
    // since alpha and beta should change by no more than a single index between
    // consecutive pivots. Eigen only supports low-rank updates to Cholesky
    // factorizations at the moment, however.

    // Compute matrix and vector views.
    SelectSubMatrixWithCovering(M, index_sets_.alpha, index_sets_.beta,
                                &M_alpha_beta_);
    SelectSubMatrixWithCovering(M, index_sets_.alpha_bar, index_sets_.beta,
                                &M_alpha_bar_beta_);
    SelectSubVector(q, index_sets_.alpha, &q_alpha_);
    SelectSubVector(q, index_sets_.alpha_bar, &q_alpha_bar_);

    // Equation (2) from [Dai 2018].
    // Note: this equation, and those below, must be kept up-to-date with
    // [Dai 2018].
    LinearSolver fMab(M_alpha_beta_); // Factorized M_alpha_beta_.
    q_prime_beta_prime_ = -fMab.Solve(q_alpha_);

    // Check whether the solution is sufficiently close. We need to do this
    // because partial pivoting LU does not estimate rank (and, from prior
    // experience in solving LCPs), loss of rank need not lead to errors in
    // solving the LCP. We assume that if the factorization is good enough to
    // solve this linear system, it's good enough to solve the subsequent
    // linear system (below). NOTE: current unit tests do not exercise the
    // affirmative evaluation of the conditional (meaning that the "return false"
    // never gets called).
    // @TODO(edrumwri) Institute a unit test that exercises the affirmative
    //   evaluation branch of the conditional when such a LCP has been
    //   identified.
    if ((M_alpha_beta_ * q_prime_beta_prime_ + q_alpha_).norm() > zero_tol)
        return false;

    // Equation (3) from [Dai 2018].
    q_prime_alpha_bar_prime_ =
        M_alpha_bar_beta_ * q_prime_beta_prime_ + q_alpha_bar_;

    // Set the components of q'.
    SetSubVector(q_prime_beta_prime_, index_sets_.beta_prime, q_prime);
    SetSubVector(q_prime_alpha_bar_prime_, index_sets_.alpha_bar_prime,
                 q_prime);

    if (output)
        std::cout << ("q': {}", q_prime->transpose()) << std::endl;
    ;

    // If it is not necessary to compute the column of M, quit now.
    if (!M_prime_col)
        return true;

    // Examine the driving variable.
    if (!indep_variables_[driving_index].is_z())
    {
        if (output)
            std::cout << ("Driving case #1: driving variable from w")
                      << std::endl;
        ;
        // Case from Section 2.2.1.
        // Determine gamma by determining the position of the driving variable
        // in INDEPENDENT W (as defined in [Dai 2018]).
        const int n = static_cast<int>(indep_variables_.size());
        int gamma = 0;
        for (int i = 0; i < n; ++i)
        {
            if (!indep_variables_[i].is_z())
            {
                if (indep_variables_[i].index() <
                    indep_variables_[driving_index].index())
                {
                    ++gamma;
                }
            }
        }

        // From Equation (4) in [Dai 2018].
        DRAKE_ASSERT(index_sets_.alpha[gamma] ==
                     indep_variables_[driving_index].index());

        // Set the unit vector.
        e_.setZero(index_sets_.beta.size());
        e_[gamma] = 1.0;

        // Equation (5).
        M_prime_driving_beta_prime_ = fMab.Solve(e_);

        // Equation (6).
        M_prime_driving_alpha_bar_prime_ =
            M_alpha_bar_beta_ * M_prime_driving_beta_prime_;
    }
    else
    {
        if (output)
            std::cout << ("Driving case #2: driving variable from z")
                      << std::endl;
        ;

        // Case from Section 2.2.2 of [Dai 2018].
        // Determine zeta.
        const int zeta = indep_variables_[driving_index].index();

        // Compute g_alpha and g_alpha_bar.
        SelectSubColumnWithCovering(M, index_sets_.alpha, zeta, &g_alpha_);
        SelectSubColumnWithCovering(M, index_sets_.alpha_bar, zeta,
                                    &g_alpha_bar_);

        // Equation (7).
        M_prime_driving_beta_prime_ = -fMab.Solve(g_alpha_);

        // Equation (8).
        M_prime_driving_alpha_bar_prime_ =
            g_alpha_bar_ + M_alpha_bar_beta_ * M_prime_driving_beta_prime_;
    }

    SetSubVector(M_prime_driving_beta_prime_, index_sets_.beta_prime,
                 M_prime_col);
    SetSubVector(M_prime_driving_alpha_bar_prime_, index_sets_.alpha_bar_prime,
                 M_prime_col);

    if (output)
        std::cout << ("M' (driving): {}", M_prime_col->transpose())
                  << std::endl;
    ;
    return true;
}

// Checks to see whether a given variable is the artificial variable.

bool UnrevisedLemkeSolver::IsArtificial(const LCPVariable &v) const
{
    const int n = static_cast<int>(dep_variables_.size());
    return v.is_z() && v.index() == n;
}

// Method for finding the index of the complement of an LCP variable in
// a tuple (strictly speaking, an unsorted vector) of independent variables.
// Aborts if the index is not found in the set or the variable is the artificial
// variable (it never should be).

int UnrevisedLemkeSolver::FindComplementIndex(const LCPVariable &query) const
{
    // Verify that the query is not the artificial variable.
    assert(!IsArtificial(query));

    const auto iter = indep_variables_indices_.find(query.Complement());
    assert(iter != indep_variables_indices_.end());
    return iter->second;
}

// Computes the solution using the current index sets. `z` must simply be
// non-null; it will be resized as necessary. Returns `true` if able to
// construct the solution and `false` if unable to find the solution to a
// necessary system of linear equations. Aborts (in LemkePivot()) if
// `artificial_index` does not correspond to the index of the artificial
// variable in the vector of independent_variables.
// @pre The artificial variable was the blocking variable, indicating that the
//      solution to the LCP can be obtained after a final pivoting operation.
// @pre `artificial_index` corresponds to the index of the artificial variable
//      in the vector of independent variables.

bool UnrevisedLemkeSolver::ConstructLemkeSolution(const tMatrixXd &M,
                                                  const tVectorXd &q,
                                                  int artificial_index,
                                                  double zero_tol,
                                                  tVectorXd *z) const
{
    assert(z);
    const int n = q.rows();

    // Compute the solution by pivoting the artificial variable, which was just
    // identified as the blocking variable, from the set of dependent variables
    // to the set of independent variables.
    tVectorXd q_prime(n);
    if (!LemkePivot(M, q, artificial_index, zero_tol, nullptr, &q_prime))
        return false;

    z->setZero(n);
    for (int i = 0; i < static_cast<int>(dep_variables_.size()); ++i)
    {
        if (dep_variables_[i].is_z())
            (*z)[dep_variables_[i].index()] = q_prime[i];
    }
    return true;
}

// Computes the blocking index using the minimum ratio test. Returns `true`
// if successful, `false` if not (due to, e.g., the driving variable being
// "unblocked" or a cycle being detected). If `false`, `blocking_index` will
// set to -1 on return.

bool UnrevisedLemkeSolver::FindBlockingIndex(const double &zero_tol,
                                             const tVectorXd &matrix_col,
                                             const tVectorXd &ratios,
                                             int *blocking_index) const
{
    assert(blocking_index);
    assert(ratios.size() == matrix_col.size());
    assert(zero_tol > 0);

    const int n = matrix_col.size();
    double min_ratio = std::numeric_limits<double>::infinity();
    *blocking_index = -1;
    for (int i = 0; i < n; ++i)
    {
        if (matrix_col[i] < -zero_tol)
        {
            if (output)
                std::cout << ("Ratio for index {}: {}", i, ratios[i])
                          << std::endl;
            ;
            if (ratios[i] < min_ratio)
            {
                min_ratio = ratios[i];
                *blocking_index = i;
            }
        }
    }

    if (*blocking_index < 0)
    {
        if (output)
            std::cout << ("driving variable is unblocked- algorithm failed")
                      << std::endl;
        ;
        return false;
    }

    // Determine all variables within the zero tolerance of the minimum ratio,
    // while simultaneously looking for the presence of the artificial variable
    // among the (possible multiple) minima.
    std::vector<int> blocking_indices;
    for (int i = 0; i < n; ++i)
    {
        if (matrix_col[i] < -zero_tol)
        {
            if (output)
                std::cout << ("Ratio for index {}: {}", i, ratios[i])
                          << std::endl;
            ;
            if (ratios[i] < min_ratio + zero_tol)
            {
                if (IsArtificial(dep_variables_[i]))
                {
                    // *Always* select the artificial variable, if multiple choices are
                    // possible ([Cottle 1992] p. 280).
                    *blocking_index = i;
                    return true;
                }
                blocking_indices.push_back(i);
            }
        }
    }

    // If there are multiple blocking variables, replace the blocking index with
    // the cycling selection.
    if (blocking_indices.size() > 1)
    {
        auto &index = selections_[indep_variables_];

        // Verify that we have not run out of indices to select, which means that
        // cycling would be occurring, in spite of cycling prevention.
        if (index >= static_cast<int>(blocking_indices.size()))
        {
            if (output)
                std::cout << ("Cycling detected- indicating failure.")
                          << std::endl;
            ;
            *blocking_index = -1;
            return false;
        }
        *blocking_index = blocking_indices[index];
        ++index;
    }

    return true;
}

bool UnrevisedLemkeSolver::IsSolution(const tMatrixXd &M, const tVectorXd &q,
                                      const tVectorXd &z, double zero_tol)
{
    using std::abs;

    const double mod_zero_tol =
        (zero_tol > 0) ? zero_tol : ComputeZeroTolerance(M);

    // Find the minima of z and w.
    const double min_z = z.minCoeff();
    const auto w = M * z + q;
    const double min_w = w.minCoeff();

    // Compute the dot product of z and w.
    const double dot = w.dot(z);
    const int n = q.size();
    return (min_z > -mod_zero_tol && min_w > -mod_zero_tol &&
            abs(dot) < 10 * n * mod_zero_tol);
}

// Note: maintainers should read Section 4.4 - 4.4.5 of [Cottle 1992] to
// understand Lemke's Algorithm (and this function).

bool UnrevisedLemkeSolver::SolveLcpLemke(const tMatrixXd &M, const tVectorXd &q,
                                         tVectorXd *z, int *num_pivots,
                                         const double &zero_tol) const
{
    using std::abs;
    using std::max;
    assert(num_pivots);

    if (output)
        std::cout
            << ("UnrevisedLemkeSolver::SolveLcpLemke() entered, M: {},  << "
                "std::endl;"
                "q: {}, ",
                M, q.transpose());

    const int n = q.size();
    const int max_pivots =
        50 * n; // O(n) pivots expected for solvable problems.

    if (M.rows() != n || M.cols() != n)
        throw std::logic_error("M's dimensions do not match that of q.");

    // Update the pivots.
    *num_pivots = 0;

    // Look for immediate exit.
    if (n == 0)
    {
        if (output)
            std::cout << ("-- LCP is zero dimensional") << std::endl;
        ;
        z->resize(0);
        return true;
    }

    // Denote the index of the artificial variable (i.e., the variable denoted
    // z₀ in [Cottle 1992], p. 266). Because z₀ is prone to being confused with
    // the first dimension of z in 0-indexed languages like C++, we refer to the
    // artificial variable as zₙ in this implementation (it is denoted zₙ₊₁ in
    // [Dai 2018], as that document uses the 1-indexing prevalent in algorithmic
    // descriptions).
    const int kArtificial = n;

    // Compute a sensible value for zero tolerance if none is given.
    double mod_zero_tol = zero_tol;
    if (mod_zero_tol <= 0)
        mod_zero_tol = ComputeZeroTolerance(M);

    // Checks to see whether the trivial solution z = 0 to the LCP w = Mz + q
    // solves the LCP. This must be the case if q is non-negative, as w would then
    // be non-negative, z would be non-negative (zero), and w'z = 0.
    if (q.minCoeff() > -mod_zero_tol)
    {
        z->setZero(q.size());
        if (output)
            std::cout << (" -- trivial solution found") << std::endl;
        ;
        if (output)
            std::cout << ("UnrevisedLemkeSolver::SolveLcpLemke() exited")
                      << std::endl;
        ;
        return true;
    }

    // Clear the cycling selections.
    selections_.clear();

    // If 'n' is identical to the size of the last problem solved, try using the
    // indices from the last problem solved.
    if (static_cast<size_t>(n) == dep_variables_.size())
    {
        // Verify that the last call found a solution (indicated by the presence
        // of the artificial variable (zn) in the independent set).
        int zn_index = -1;
        for (int i = 0;
             i < static_cast<int>(indep_variables_.size()) && zn_index < 0; ++i)
        {
            if (IsArtificial(indep_variables_[i]))
                zn_index = i;
        }

        if (zn_index >= 0)
        {
            // Compute the candidate solution.
            if (ConstructLemkeSolution(M, q, zn_index, mod_zero_tol, z))
            {
                if (IsSolution(M, q, *z, mod_zero_tol))
                {
                    // If z truly is the solution, return now, indicating only one pivot
                    // (in the solution construction) was performed.
                    ++(*num_pivots);
                    return true;
                }
            }
            else
            {
                if (output)
                    std::cout
                        << ("Failed to solve linear system implied by last  "
                            "<< std::endl;"
                            "solution");
            }
        }
    }

    // Set the LCP variables. Start with all z variables independent and all w
    // variables dependent.
    indep_variables_.resize(n + 1);
    dep_variables_.resize(n);
    for (int i = 0; i < n; ++i)
    {
        dep_variables_[i] = LCPVariable(false, i);
        indep_variables_[i] = LCPVariable(true, i);
    }
    // z needs one more variable (the artificial variable), whose index we
    // denote as n to keep it from corresponding to any actual vector index.
    indep_variables_[n] = LCPVariable(true, n);

    // Compute zn*, the smallest value of the artificial variable zn for which
    // w = q + zn >= 0. Let blocking denote a component of w that equals
    // zero when zn = zn*.
    int blocking_index = -1;
    bool blocking_index_found =
        FindBlockingIndex(mod_zero_tol, q, q, &blocking_index);
    assert(blocking_index_found);

    // Pivot blocking, artificial. Note that we rely upon the dependent variables
    // being ordered sequentially in both arrays.
    LCPVariable blocking = dep_variables_[blocking_index];
    int driving_index = blocking.index();
    std::swap(dep_variables_[blocking_index], indep_variables_[kArtificial]);
    if (output)
        std::cout << ("First blocking variable {}{}",
                      ((blocking.is_z()) ? "z" : "w"), blocking.index())
                  << std::endl;
    if (output)
        std::cout << ("First driving variable (artificial)") << std::endl;
    ;

    // Initialize the independent variable indices. We do this after the initial
    // variable swap for simplicity.
    for (int i = 0; i < static_cast<int>(indep_variables_.size()); ++i)
        indep_variables_indices_[indep_variables_[i]] = i;

    // Output the independent and dependent variable tuples.
    auto to_string = [](const std::vector<LCPVariable> &vars) -> std::string {
        std::ostringstream oss;
        for (int i = 0; i < static_cast<int>(vars.size()); ++i)
            oss << ((vars[i].is_z()) ? "z" : "w") << vars[i].index() << " ";
        return oss.str();
    };
    // unused(to_string); // ... when in release mode.
    if (output)
        std::cout << ("Independent set variables: {}",
                      to_string(indep_variables_))
                  << std::endl;
    ;
    if (output)
        std::cout << ("Dependent set variables: {}", to_string(dep_variables_))
                  << std::endl;
    ;

    // Pivot up to the maximum number of times.
    tVectorXd q_prime(n), M_prime_col(n);
    while (++(*num_pivots) < max_pivots)
    {
        if (output)
            std::cout << ("New driving variable {}{}",
                          ((indep_variables_[driving_index].is_z()) ? "z"
                                                                    : "w"),
                          indep_variables_[driving_index].index())
                      << std::endl;

        // Compute the permuted q and driving column of the permuted M matrix.
        if (!LemkePivot(M, q, driving_index, mod_zero_tol, &M_prime_col,
                        &q_prime))
        {
            if (output)
                std::cout << ("Linear system solve failed.") << std::endl;
            ;
            z->setZero(n);
            return false;
        }

        // Find the blocking variable.
        if (!FindBlockingIndex(
                mod_zero_tol, M_prime_col,
                -(q_prime.array() / M_prime_col.array()).matrix(),
                &blocking_index))
        {
            z->setZero(n);
            return false;
        }
        blocking = dep_variables_[blocking_index];
        if (output)
            std::cout << ("Blocking variable {}{}",
                          ((blocking.is_z()) ? "z" : "w"), blocking.index())
                      << std::endl;
        ;

        // See whether the artificial variable blocks the driving variable.
        if (blocking.index() == kArtificial)
        {
            assert(blocking.is_z());

            // Pivot zn with the driving variable.
            std::swap(dep_variables_[blocking_index],
                      indep_variables_[driving_index]);

            // Compute the permuted q, and convert it into a solution.
            if (ConstructLemkeSolution(M, q, driving_index, mod_zero_tol, z))
            {
                if (IsSolution(M, q, *z))
                    return true;

                if (output)
                    std::cout
                        << ("Solution not computed to requested tolerance")
                        << std::endl;
                ;
                z->setZero(n);
                return false;
            }

            // Otherwise, indicate failure.
            std::cout
                << ("Linear system solver failed to construct Lemke solution");
            z->setZero(n);
            return false;
        }

        // Pivot the blocking variable and the driving variable.
        std::swap(dep_variables_[blocking_index],
                  indep_variables_[driving_index]);

        // Update the index map.
        auto indep_variables_indices_iter =
            indep_variables_indices_.find(dep_variables_[blocking_index]);
        indep_variables_indices_.erase(indep_variables_indices_iter);
        indep_variables_indices_[indep_variables_[driving_index]] =
            driving_index;

        // Make the driving variable the complement of the blocking variable.
        driving_index = FindComplementIndex(blocking);

        if (output)
            std::cout << ("Independent set variables: {}",
                          to_string(indep_variables_))
                      << std::endl;
        if (output)
            std::cout << ("Dependent set variables: {}",
                          to_string(dep_variables_))
                      << std::endl;
        ;
    }

    // If here, the maximum number of pivots has been exceeded.
    z->setZero(n);
    if (output)
        std::cout << ("Maximum number of pivots exceeded") << std::endl;
    ;
    return false;
}

UnrevisedLemkeSolver::UnrevisedLemkeSolver()
// : SolverBase(&id, &is_available, &is_enabled, &ProgramAttributesSatisfied)
{
}

UnrevisedLemkeSolver::~UnrevisedLemkeSolver() {}

// SolverId UnrevisedLemkeSolverId::id()
// {
//     static const never_destroyed<SolverId> singleton{"Unrevised Lemke"};
//     return singleton.access();
// }

//  SolverId UnrevisedLemkeSolver::id()
// {
//     return UnrevisedLemkeSolverId::id();
// }

bool UnrevisedLemkeSolver::is_available() { return true; }

bool UnrevisedLemkeSolver::is_enabled() { return true; }

//
// bool UnrevisedLemkeSolver::ProgramAttributesSatisfied(
//     const MathematicalProgram &prog)
// {
//     // This solver imposes restrictions that its problem:
//     //
//     // (1) Contains only linear complementarity constraints,
//     // (2) Has no element of any decision variable appear in more than one
//     //     constraint, and
//     // (3) Has every element of every decision variable in a constraint.
//     //
//     // Restriction 1 could reasonably be relaxed by reformulating other
//     // constraint types that can be expressed as LCPs (eg, convex QLPs),
//     // although this would also entail adding an output stage to convert
//     // the LCP results back to the desired form.  See eg. @RussTedrake on
//     // how to convert a linear equality constraint of n elements to an
//     // LCP of 2n elements.
//     //
//     // There is no obvious way to relax restriction 2.
//     //
//     // Restriction 3 could reasonably be relaxed to simply let unbound
//     // variables sit at 0.
//     if (prog.required_capabilities() !=
//         ProgramAttributes({ProgramAttribute::kLinearComplementarityConstraint}))
//     {
//         return false;
//     }

//     // Check that the available LCPs cover the program and no two LCPs cover the
//     // same variable.
//     const auto &bindings = prog.linear_complementarity_constraints();
//     for (int i = 0; i < static_cast<int>(prog.num_vars()); ++i)
//     {
//         int coverings = 0;
//         for (const auto &binding : bindings)
//         {
//             if (binding.ContainsVariable(prog.decision_variable(i)))
//             {
//                 coverings++;
//             }
//         }
//         if (coverings != 1)
//         {
//             return false;
//         }
//     }

//     return true;
// }

// } // namespace solvers
// } // namespace drake

// DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
//     class ::drake::solvers::UnrevisedLemkeSolver)
