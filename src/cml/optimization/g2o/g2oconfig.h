#include <g2o/types/sba/types_sba.h>
#include <g2o/types/sba/vertex_se3_expmap.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>

#ifdef ANDROID
template <typename T> using DefaultG2OSolverWithCovariance = g2o::LinearSolverEigen<T>;

template <typename T> using DefaultG2OSolverForSpeed = g2o::LinearSolverEigen<T>;

using DefaultG2OSolverForStructureOnly = g2o::StructureOnlySolver<3>;

#else
//#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
template <typename T> using DefaultG2OSolverWithCovariance = g2o::LinearSolverEigen<T>;

template <typename T> using DefaultG2OSolverForSpeed = g2o::LinearSolverEigen<T>;

using DefaultG2OSolverForStructureOnly = g2o::StructureOnlySolver<3>;
#endif