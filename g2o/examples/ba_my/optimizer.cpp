//
// Created by s on 28.05.20.
//

#include <g2o/core/block_solver.h>
#include "optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> getSolver(bool DENSE) {
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    if (DENSE) {
        linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } else {
        linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    }
    return linearSolver;
}


std::shared_ptr<g2o::SparseOptimizer> getOptimizer(bool DENSE, bool verbose) {
    auto  optimizer = std::make_shared<g2o::SparseOptimizer>();
    optimizer->setVerbose(verbose);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver = getSolver(DENSE);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(move(linearSolver))
    );
    optimizer->setAlgorithm(solver);
    return optimizer;
}
