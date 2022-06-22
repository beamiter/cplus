#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#include <Eigen/src/Core/DiagonalMatrix.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <tuple>
#include <vector>

#include "base/base.h"
#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "trajectory_optimization/cost_functions.h"

using Eigen::DiagonalMatrix;
using Eigen::MatrixX;

class AbstractObjective {
public:
  virtual ~AbstractObjective() = default;
  // Pure Virtual Function
  virtual int length() const = 0;
  virtual int state_dim(int k) const = 0;
  virtual int control_dim(int k) const = 0;
  virtual std::vector<double> get_J() const = 0;
};

template <typename C> class Objective : public AbstractObjective {
  // For template type check.
  static_assert(std::is_base_of<CostFunction, C>::value,
                "T is not derived of CostFunction");

public:
  // Constructors
  Objective(std::vector<C> cost_in,
            DiffMethod diff_method_in = DiffMethod::UserDefined) {
    cost_ = cost_in;
    int N = cost_.size();
    J.resize(N, 0);
    const_grad.resize(N, false);
    const_hess.resize(N, false);
    diff_method.resize(N, diff_method_in);
  }

  Objective(std::vector<C> cost_in, std::vector<DiffMethod> diff_method_in) {
    cost_ = cost_in;
    int N = cost_.size();
    J.resize(N, 0);
    const_grad.resize(N, false);
    const_hess.resize(N, false);
    diff_method = diff_method_in;
  }

  static Objective<C> init(C cost_in, int N) {
    return Objective<C>(std::vector<C>(N, cost_in));
  }

  static Objective<C> init(C cost_in, C cost_terminal, int N) {
    std::vector<C> cost_tmp;
    cost_tmp.resize(N - 1, cost_in);
    cost_tmp.push_back(cost_terminal);
    return Objective<C>(cost_tmp);
  }

  static Objective<C> init(std::vector<C> cost_in, C cost_terminal) {
    std::vector<C> cost_tmp(cost_in);
    cost_tmp.push_back(cost_terminal);
    return Objective<C>(cost_tmp);
  }

  auto begin() { return cost_.begin(); }
  auto end() { return cost_.end(); }
  auto size() { return cost_.size(); }
  auto front() { return cost_.front(); }
  auto back() { return cost_.back(); }
  auto &operator[](int k) { return cost_[k]; }
  const auto &operator[](int k) const { return cost_[k]; }

  // Overrides.
  int length() const override { return cost_.size(); }
  int state_dim(int k) const override { return cost_[k].state_dim(); }
  int control_dim(int k) const override { return cost_[k].control_dim(); }
  std::vector<double> get_J() const override { return J; }

  std::tuple<std::vector<int>, std::vector<int>> dims() {
    std::vector<int> nx, nu;
    for (const auto &c : cost) {
      nx.push_back(c.state_dim());
      nu.push_back(c.control_dim());
    }
    return std::make_tuple(nx, nu);
  }
  std::tuple<int, int, int> dims(int k) {
    return std::make_tuple(cost_[k].state_dim(), cost_[k].control_dim(),
                           cost_.size());
  }
  bool is_quadratic() {
    return std::all_of(const_hess.begin(), const_hess.end(),
                       [](const auto hess) { return hess == true; });
  }
  template <typename KP> void cost_inplace(const SampledTrajectory<KP> &Z) {
    for (int k = 0; k < length(); ++k) {
      evaluate(cost_[k], J[k], Z.data);
    }
  }
  template <typename KP> double cost(const SampledTrajectory<KP> &Z) {
    cost_inplace(Z);
    const auto J = get_J();
    return std::accumulate(J.begin(), J.end(), 0);
  }
  template <typename KP> double cost(const SampledTrajectory<KP> &Z) const {
    double J = 0.0;
    for (int k = 0; k < length(); ++k) {
      J += evaluate<KP>(&cost_[k], Z[k]);
    }
    return J;
  }

  // Members
  std::vector<C> cost_;
  std::vector<double> J;
  std::vector<bool> const_grad;
  std::vector<bool> const_hess;
  std::vector<DiffMethod> diff_method;
};

template <int n, int m, typename T>
Objective<QuadraticCostS<n, m, T>>
LQRObjective(const MatrixX<T> &Q, const MatrixX<T> &R, const MatrixX<T> &Qf,
             const VectorX<T> &xf, const VectorX<T> &uf, int N,
             bool checks = true,
             DiffMethod diffmethod = DiffMethod::UserDefined) {
  assert(Q.rows() == xf.size());
  assert(Qf.rows() == xf.size());
  assert(R.rows() == uf.size());
  assert(n == Q.rows());
  assert(m == R.rows());
  const auto &H = MatrixX<T>::Zero(m, n);
  const auto &q = -1.0 * Q * xf;
  const auto &r = -1.0 * R * uf;
  double c = 0.5 * xf.adjoint() * Q * xf;
  c += 0.5 * uf.adjoint() * R * uf;
  const auto &qf = -1.0 * Qf * xf;
  const double cf = 0.5 * xf.adjoint() * Qf * xf;

  const auto &l = QuadraticCostS<n, m, T>(Q, R, H, q, r, c, checks, false);
  const auto &lN = QuadraticCostS<n, m, T>(Qf, R, H, qf, r, cf, false, true);
  return Objective<QuadraticCostS<n, m, T>>::init(l, lN, N);
}
template <int n, int m, typename T>
Objective<DiagonalCostS<n, m, T>>
LQRObjective(const DiagonalMatrix<T, n> &Q, const DiagonalMatrix<T, m> &R,
             const DiagonalMatrix<T, n> &Qf, const VectorX<T> &xf,
             const VectorX<T> &uf, int N, bool checks = true,
             DiffMethod diffmethod = DiffMethod::UserDefined) {
  assert(Q.rows() == xf.size());
  assert(Qf.rows() == xf.size());
  assert(R.rows() == uf.size());
  assert(n == Q.rows());
  assert(m == R.rows());
  const auto &q = -1.0 * Q * xf;
  const auto &r = -1.0 * R * uf;
  double c = 0.5 * xf.adjoint() * Q * xf;
  c += 0.5 * uf.adjoint() * R * uf;
  const auto &qf = -1.0 * Qf * xf;
  const auto &cf = 0.5 * xf.adjoint() * Qf * xf;

  const auto &l = DiagonalCostS<n, m, T>(Q, R, q, r, c, checks, false);
  const auto &lN = DiagonalCostS<n, m, T>(Qf, R, qf, r, cf, false, true);
  return Objective<DiagonalCostS<n, m, T>>::init(l, lN, N);
}

#endif
