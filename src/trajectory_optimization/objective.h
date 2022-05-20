#ifndef OBJECTIVE_H
#define OBJECTIVE_H

#include <algorithm>
#include <tuple>
#include <vector>

#include "base/base.h"
#include "robot_dynamics/functionbase.h"
#include "trajectory_optimization/cost_functions.h"

using Eigen::MatrixX;

class AbstractObjective {
public:
  // Pure Virtual Function
  virtual int length() const = 0;
  virtual int state_dim(int k) const = 0;
  virtual int control_dim(int k) const = 0;
  virtual std::vector<double> get_J() const = 0;
};

// template<>
// auto length(AbstractObjective obj) {
//   return length(obj.cost);
// }

template <typename C> class Objective : AbstractObjective {
public:
  // Constructors
  Objective(std::vector<C> cost_in, DiffMethod diff_method_in) {
    cost = cost_in;
    int N = cost.size();
    J.resize(N, 0);
    const_grad.resize(N, false);
    const_hess.resize(N, false);
    diff_method.resize(N, diff_method_in);
  }
  Objective(std::vector<C> cost_in, std::vector<DiffMethod> diff_method_in) {
    cost = cost_in;
    int N = cost.size();
    J.resize(N, 0);
    const_grad.resize(N, false);
    const_hess.resize(N, false);
    diff_method = diff_method_in;
  }
  Objective(C cost_in, int N, ...) { cost.resize(N, cost_in); }
  Objective(C cost_in, C cost_terminal, int N, ...) {
    cost.resize(N - 1, cost_in);
    cost.push_back(cost_terminal);
  }
  Objective(std::vector<C> cost_in, C cost_terminal, ...) {
    int N = cost.size() + 1;
    cost = cost_in;
    cost.push_back(cost_terminal);
  }

  auto begin() { return cost.begin(); }
  auto end() { return cost.end(); }
  auto size() { return cost.size(); }
  auto front() { return cost.front(); }
  auto back() { return cost.back(); }
  auto &operator[](int k) { return cost[k]; }
  const auto &operator[](int k) const { return cost[k]; }

  int length() const override { return cost.size(); }
  int state_dim(int k) const override { return cost[k].state_dim(); }
  int control_dim(int k) const override { return cost[k].control_dim(); }
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
    return std::make_tuple(cost[k].state_dim(), cost[k].control_dim(),
                           cost.size());
  }

  bool is_quadratic() {
    return std::all_of(const_hess.begin(), const_hess.end(),
                       [](const auto hess) { return hess == true; });
  }

  // Members
  std::vector<C> cost;
  std::vector<double> J;
  std::vector<bool> const_grad;
  std::vector<bool> const_hess;
  std::vector<DiffMethod> diff_method;
};

template <typename C, typename T>
Objective<C> LQRObjective(MatrixX<T> Q, MatrixX<T> R, MatrixX<T> Qf,
                          VectorX<T> xf, VectorX<T> uf, int N,
                          bool checks = true,
                          DiffMethod diffmethod = DiffMethod::UserDefined) {
  assert(Q.size() == xf.size());
  assert(Qf.size() == xf.size());
  assert(R.size() == uf.size());
  const int n = Q.size();
  const int m = R.size();
  const auto &H = MatrixX<T>::Zero(m, n);
  const auto &q = -Q * xf;
  const auto &r = -R * uf;
  const double c = 0.5 * xf.adjoint() * Q * xf + 0.5 * uf.adjoint() * uf;
  const auto &qf = -Qf * xf;
  const double cf = 0.5 * xf.adjoint() * Qf * xf;

  const auto &l = QuadraticCost<n, m, T, Inplace, EuclideanState>(
      Q, R, H, q, r, c, checks = checks);
  const auto &lN = QuadraticCost<n, m, T, Inplace, EuclideanState>(
      Qf, R, H, qf, r, cf, false, true);
  return Objective<double>(l, lN, N);
}

#endif
