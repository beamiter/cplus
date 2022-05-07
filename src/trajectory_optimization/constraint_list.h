#ifndef CONSTRAINT_LIST_H
#define CONSTRAINT_LIST_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "abstract_constraint.h"
#include "robot_dynamics/discrete_dynamics.h"

struct AbstractConstraintSet {};

struct ConstraintList : AbstractConstraintSet {
  std::vector<int> nx;
  std::vector<int> nu;
  std::vector<AbstractConstraint> constraints;
  std::vector<Eigen::ArithmeticSequence<int, int>> inds;
  std::vector<FunctionSignature> sigs;
  std::vector<DiffMethod> diffs;
  std::vector<int> p;
  ConstraintList(std::vector<int> nx_in, std::vector<int> nu_in)
      : nx(std::move(nx_in)), nu(std::move(nu_in)) {
    std::cout << "here\n";
    auto N = length(nx);
    this->p = std::vector<int>(N, 0);
  }
  ConstraintList(int n, int m, int N) {
    std::cout << "here\n";
    this->nx = std::vector<int>(n, N);
    this->nu = std::vector<int>(m, N);
    ConstraintList(this->nx, this->nu);
  }
  ConstraintList(std::vector<DiscreteDynamics> models) {
    std::cout << "here\n";
    std::tie(this->nx, this->nu) = dims(models);
    ConstraintList(this->nx, this->nu);
  }
};

#endif
