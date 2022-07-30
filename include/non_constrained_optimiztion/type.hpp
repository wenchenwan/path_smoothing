//
// Created by yangt on 18-12-21.
//

#ifndef PATH_SMOOTHING_TYPE_HPP
#define PATH_SMOOTHING_TYPE_HPP

#include <glog/logging.h>
namespace ncopt {

enum LinearSolverType {
  // These solvers are for general rectangular systems formed from the
  // normal equations A'A x = A'b. They are direct solvers and do not
  // assume any special problem structure.

  // Solve the normal equations using a dense Cholesky solver; based
  // on Eigen.
          DENSE_NORMAL_CHOLESKY,

  // Solve the normal equations using a dense QR solver; based on
  // Eigen.
          DENSE_QR,

  // Solve the normal equations using a sparse cholesky solver; requires
  // SuiteSparse or CXSparse.
          SPARSE_NORMAL_CHOLESKY,

  // Specialized solvers, specific to problems with a generalized
  // bi-partitite structure.

  // Solves the reduced linear system using a dense Cholesky solver;
  // based on Eigen.
          DENSE_SCHUR,

  // Solves the reduced linear system using a sparse Cholesky solver;
  // based on CHOLMOD.
          SPARSE_SCHUR,

  // Solves the reduced linear system using Conjugate Gradients, based
  // on a new Ceres implementation.  Suitable for large scale
  // problems.
          ITERATIVE_SCHUR,

  // Conjugate gradients on the normal equations.
          CGNR
};

enum PreconditionerType {
  // Trivial preconditioner - the identity matrix.
          IDENTITY,

  // Block diagonal of the Gauss-Newton Hessian.
          JACOBI,

  // Note: The following three preconditioners can only be used with
  // the ITERATIVE_SCHUR solver. They are well suited for Structure
  // from Motion problems.

  // Block diagonal of the Schur complement. This preconditioner may
  // only be used with the ITERATIVE_SCHUR solver.
          SCHUR_JACOBI,

  // Visibility clustering based preconditioners.
  //
  // The following two preconditioners use the visibility structure of
  // the scene to determine the sparsity structure of the
  // preconditioner. This is done using a clustering algorithm. The
  // available visibility clustering algorithms are described below.
  //
  // Note: Requires SuiteSparse.
          CLUSTER_JACOBI,
  CLUSTER_TRIDIAGONAL
};

enum VisibilityClusteringType {
  // Canonical views algorithm as described in
  //
  // "Scene Summarization for Online Image Collections", Ian Simon, Noah
  // Snavely, Steven M. Seitz, ICCV 2007.
  //
  // This clustering algorithm can be quite slow, but gives high
  // quality clusters. The original visibility based clustering paper
  // used this algorithm.
          CANONICAL_VIEWS,

  // The classic single linkage algorithm. It is extremely fast as
  // compared to CANONICAL_VIEWS, but can give slightly poorer
  // results. For problems with large number of cameras though, this
  // is generally a pretty good option.
  //
  // If you are using SCHUR_JACOBI preconditioner and have SuiteSparse
  // available, CLUSTER_JACOBI and CLUSTER_TRIDIAGONAL in combination
  // with the SINGLE_LINKAGE algorithm will generally give better
  // results.
          SINGLE_LINKAGE
};

enum SparseLinearAlgebraLibraryType {
  // High performance sparse Cholesky factorization and approximate
  // minimum degree ordering.
          SUITE_SPARSE,

  // A lightweight replacement for SuiteSparse, which does not require
  // a LAPACK/BLAS implementation. Consequently, its performance is
  // also a bit lower than SuiteSparse.
          CX_SPARSE,

  // Eigen's sparse linear algebra routines. In particular Ceres uses
  // the Simplicial LDLT routines.
          EIGEN_SPARSE,

  // Apple's Accelerate framework sparse linear algebra routines.
          ACCELERATE_SPARSE,

  // No sparse linear solver should be used.  This does not necessarily
  // imply that Ceres was built without any sparse library, although that
  // is the likely use case, merely that one should not be used.
          NO_SPARSE
};

enum DenseLinearAlgebraLibraryType {
  EIGEN,
  LAPACK
};

// Logging options
// The options get progressively noisier.
enum LoggingType {
  SILENT,
  PER_MINIMIZER_ITERATION
};

enum MinimizerType {
  LINE_SEARCH,
  TRUST_REGION
};

enum LineSearchDirectionType {
  // Negative of the gradient.
  STEEPEST_DESCENT,

  // A generalization of the Conjugate Gradient method to non-linear
  // functions. The generalization can be performed in a number of
  // different ways, resulting in a variety of search directions. The
  // precise choice of the non-linear conjugate gradient algorithm
  // used is determined by NonlinerConjuateGradientType.
  NONLINEAR_CONJUGATE_GRADIENT,

  BFGS,
  LBFGS,

};

// Nonlinear conjugate gradient methods are a generalization of the
// method of Conjugate Gradients for linear systems. The
// generalization can be carried out in a number of different ways
// leading to number of different rules for computing the search
// direction. Ceres provides a number of different variants. For more
// details see Numerical Optimization by Nocedal & Wright.
enum NonlinearConjugateGradientType {
  FLETCHER_REEVES,
  POLAK_RIBIERE,
  POLAK_RIBIERE_PLUS,
  FR_PR,
};


enum LineSearchType {
  // Backtracking line search with polynomial interpolation or
  // bisection.
          ARMIJO,
  WOLFE,
};

// Ceres supports different strategies for computing the trust region
// step.
enum TrustRegionStrategyType {
  // The default trust region strategy is to use the step computation
  // used in the Levenberg-Marquardt algorithm. For more details see
  // levenberg_marquardt_strategy.h
          LEVENBERG_MARQUARDT,

  // Powell's dogleg algorithm interpolates between the Cauchy point
  // and the Gauss-Newton step. It is particularly useful if the
  // LEVENBERG_MARQUARDT algorithm is making a large number of
  // unsuccessful steps. For more details see dogleg_strategy.h.
  //
  // NOTES:
  //
  // 1. This strategy has not been experimented with or tested as
  // extensively as LEVENBERG_MARQUARDT, and therefore it should be
  // considered EXPERIMENTAL for now.
  //
  // 2. For now this strategy should only be used with exact
  // factorization based linear solvers, i.e., SPARSE_SCHUR,
  // DENSE_SCHUR, DENSE_QR and SPARSE_NORMAL_CHOLESKY.
          DOGLEG
};

// Ceres supports two different dogleg strategies.
// The "traditional" dogleg method by Powell and the
// "subspace" method described in
// R. H. Byrd, R. B. Schnabel, and G. A. Shultz,
// "Approximate solution of the trust region problem by minimization
//  over two-dimensional subspaces", Mathematical Programming,
// 40 (1988), pp. 247--263
enum DoglegType {
  // The traditional approach constructs a dogleg path
  // consisting of two line segments and finds the furthest
  // point on that path that is still inside the trust region.
          TRADITIONAL_DOGLEG,

  // The subspace approach finds the exact minimum of the model
  // constrained to the subspace spanned by the dogleg path.
          SUBSPACE_DOGLEG
};

enum TerminationType {
  // Minimizer terminated because one of the convergence criterion set
  // by the user was satisfied.
  //
  // 1.  (new_cost - old_cost) < function_tolerance * old_cost;
  // 2.  max_i |gradient_i| < gradient_tolerance
  // 3.  |step|_2 <= parameter_tolerance * ( |x|_2 +  parameter_tolerance)
  //
  // The user's parameter blocks will be updated with the solution.
          COST_DECREMENT_CONVERGENCE,

  GRADIENT_NORM_CONVERGENCE,

  STEP_NORM_CONVERGENCE,

  // The solver ran for maximum number of iterations or maximum amount
  // of time specified by the user, but none of the convergence
  // criterion specified by the user were met. The user's parameter
  // blocks will be updated with the solution found so far.
          NO_CONVERGENCE,

  // The minimizer terminated because of an error.  The user's
  // parameter blocks will not be updated.
          FAILURE,

  // Using an IterationCallback object, user code can control the
  // minimizer. The following enums indicate that the user code was
  // responsible for termination.
  //
  // Minimizer terminated successfully because a user
  // IterationCallback returned SOLVER_TERMINATE_SUCCESSFULLY.
  //
  // The user's parameter blocks will be updated with the solution.
          USER_SUCCESS,

  // Minimizer terminated because because a user IterationCallback
  // returned SOLVER_ABORT.
  //
  // The user's parameter blocks will not be updated.
          USER_FAILURE,

  ITERATION_COUNT_LIMITED,

  NO_AVILIABLE_STEP_LENGTH
};

// Enums used by the IterationCallback instances to indicate to the
// solver whether it should continue solving, the user detected an
// error or the solution is good enough and the solver should
// terminate.
enum CallbackReturnType {
  // Continue solving to next iteration.
          SOLVER_CONTINUE,

  // Terminate solver, and do not update the parameter blocks upon
  // return. Unless the user has set
  // Solver:Options:::update_state_every_iteration, in which case the
  // state would have been updated every iteration
  // anyways. Solver::Summary::termination_type is set to USER_ABORT.
          SOLVER_ABORT,

  // Terminate solver, update state and
  // return. Solver::Summary::termination_type is set to USER_SUCCESS.
          SOLVER_TERMINATE_SUCCESSFULLY
};

// The format in which linear least squares problems should be logged
// when Solver::Options::lsqp_iterations_to_dump is non-empty.
enum DumpFormatType {
  // Print the linear least squares problem in a human readable format
  // to stderr. The Jacobian is printed as a dense matrix. The vectors
  // D, x and f are printed as dense vectors. This should only be used
  // for small problems.
          CONSOLE,

  // Write out the linear least squares problem to the directory
  // pointed to by Solver::Options::lsqp_dump_directory as text files
  // which can be read into MATLAB/Octave. The Jacobian is dumped as a
  // text file containing (i,j,s) triplets, the vectors D, x and f are
  // dumped as text files containing a list of their values.
  //
  // A MATLAB/octave script called lm_iteration_???.m is also output,
  // which can be used to parse and load the problem into memory.
          TEXTFILE
};

// For SizedCostFunction and AutoDiffCostFunction, DYNAMIC can be
// specified for the number of residuals. If specified, then the
// number of residuas for that cost function can vary at runtime.
enum DimensionType {
  DYNAMIC = -1
};

// The differentiation method used to compute numerical derivatives in
// NumericDiffCostFunction and DynamicNumericDiffCostFunction.
enum NumericDiffMethodType {
  // Compute central finite difference: f'(x) ~ (f(x+h) - f(x-h)) / 2h.
          CENTRAL,

  // Compute forward finite difference: f'(x) ~ (f(x+h) - f(x)) / h.
          FORWARD,

  // Adaptive numerical differentiation using Ridders' method. Provides more
  // accurate and robust derivatives at the expense of additional cost
  // function evaluations.
          RIDDERS
};

enum LineSearchInterpolationType {
  BISECTION,
  QUADRATIC,
  CUBIC
};

enum CovarianceAlgorithmType {
  DENSE_SVD,
  SPARSE_QR,
};

extern const char *TerminationTypeToString(const TerminationType &type);
extern const char *LineSearchDirectionTypeToString(const LineSearchDirectionType &type);
extern const char *LIneSearchStepTypeToString(const LineSearchType &type);

}
#endif //PATH_SMOOTHING_TYPE_HPP
