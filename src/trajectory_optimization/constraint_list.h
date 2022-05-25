#ifndef CONSTRAINT_LIST_H
#define CONSTRAINT_LIST_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "abstract_constraint.h"
#include "robot_dynamics/discrete_dynamics.h"

class AbstractConstraintSet {
public:
  virtual ~AbstractConstraintSet() = default;
};

class ConstraintList : public AbstractConstraintSet {
public:
  std::vector<int> nx;
  std::vector<int> nu;
  std::vector<AbstractConstraint> constraints;
  std::vector<Eigen::ArithmeticSequence<int, int>> inds;
  std::vector<FunctionSignature> sigs;
  std::vector<DiffMethod> diffs;
  std::vector<int> p;
  ConstraintList(std::vector<int> nx_in, std::vector<int> nu_in)
      : nx(std::move(nx_in)), nu(std::move(nu_in)) {
    const auto N = nx.size();
    this->p = std::vector<int>(N, 0);
  }
  ConstraintList(int n, int m, int N) {
    this->nx = std::vector<int>(n, N);
    this->nu = std::vector<int>(m, N);
    ConstraintList(this->nx, this->nu);
  }
  ConstraintList(const std::vector<const DiscreteDynamics *> &models) {
    std::tie(this->nx, this->nu) = dims(models);
    ConstraintList(this->nx, this->nu);
  }
};

#endif
