#ifndef DYNAMICS_EXPANSION_H
#define DYNAMICS_EXPANSION_H

#include <Eigen/Dense>

using Eigen::VectorX;
using Eigen::MatrixX;

template <typename T>
struct DynamicsExpansion {
  VectorX<T> f;
  MatrixX<T> df;
};

#endif
