//
// Created by s on 28.05.20.
//

#ifndef G2O_OPTIMIZER_H
#define G2O_OPTIMIZER_H
#include <memory>
#include <g2o/core/sparse_optimizer.h>

std::shared_ptr<g2o::SparseOptimizer> getOptimizer(bool DENSE, bool verbose);

#endif //G2O_OPTIMIZER_H
