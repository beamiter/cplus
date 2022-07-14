#ifndef ILQR_COST_EXPANTION
#define ILQR_COST_EXPANTION

#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>

#include "base/base.h"
#include "ilqr/dynamics_expansion.h"
#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/statevectortype.h"

using Eigen::all;
using Eigen::last;
using Eigen::MatrixX;
using Eigen::Ref;
using Eigen::seq;
using Eigen::VectorX;

// State and control
template <typename T, bool B> class StateControlExpansion {};
template <typename T> class StateControlExpansion<T, true> {
public:
  static constexpr bool state_control = true;
  static_assert(state_control, "For state and control!!!");

  StateControlExpansion(int n, int m)
      : ix(0, n), data(MatrixX<T>::Zero(n + m, n + m + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)), iu(n, m), ux(data(iu, ix)),
        uu(data(iu, iu)), u(grad(iu)) {
    hess.setIdentity();
    CHECK(n > 0 && m > 0);
  }

  Eigen::ArithmeticSequence<int, int> ix;
  MatrixX<T> data;
  Ref<MatrixX<T>> hess;
  Ref<VectorX<T>> grad;
  Ref<MatrixX<T>> xx;
  Ref<VectorX<T>> x;
  Eigen::ArithmeticSequence<int, int> iu;
  Ref<MatrixX<T>> ux;
  Ref<MatrixX<T>> uu;
  Ref<VectorX<T>> u;
};
template <typename T> class StateControlExpansion<T, false> {
public:
  static constexpr bool state_control = false;
  static_assert(!state_control, "For only state!!!");

  // m is not essential
  StateControlExpansion(int n)
      : ix(0, n), data(MatrixX<T>::Zero(n, n + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)) {
    hess.setIdentity();
    CHECK(n > 0);
  }

  Eigen::ArithmeticSequence<int, int> ix;
  MatrixX<T> data;
  Ref<MatrixX<T>> hess;
  Ref<VectorX<T>> grad;
  Ref<MatrixX<T>> xx;
  Ref<VectorX<T>> x;
};

template <typename T, bool B = true> class CostExpansion {
public:
  static const bool state_control = B;
  CostExpansion() = default;
  CostExpansion(const std::vector<int> &nx, const std::vector<int> &nu = {}) {
    const auto N = nx.size();
    const_hess_.resize(N, false);
    const_grad_.resize(N, false);
    for (auto k = 0; k < N; ++k) {
      expansion_vec_.push_back(
          std::make_unique<StateControlExpansion<T, B>>(nx[k], nu[k]));
    }
  }
  CostExpansion(int n, int m, int N) {
    const_hess_.resize(N, false);
    const_grad_.resize(N, false);
    for (auto k = 0; k < N; ++k) {
      expansion_vec_.push_back(
          std::make_unique<StateControlExpansion<T, B>>(n, m));
    }
  }
  CostExpansion(const CostExpansion &in)
      : expansion_vec_(in.expansion_vec_), const_hess_(in.const_hess_),
        const_grad_(in.const_grad_) {}

  const auto &operator[](int i) const { return expansion_vec_[i]; }
  auto &operator[](int i) { return expansion_vec_[i]; }
  const auto &at(int i) const { return expansion_vec_[i]; }
  auto &at(int i) { return expansion_vec_[i]; }
  const auto &front() const { return expansion_vec_.front(); }
  auto &front() { return expansion_vec_.front(); }
  const auto &back() const { return expansion_vec_.back(); }
  auto &back() { return expansion_vec_.back(); }
  int size() const { return expansion_vec_.size(); }
  int length() const { return expansion_vec_.size(); }

  std::vector<std::unique_ptr<StateControlExpansion<T, B>>> expansion_vec_;
  std::vector<bool> const_hess_;
  std::vector<bool> const_grad_;
};

template <typename KP, bool B>
std::shared_ptr<CostExpansion<typename KP::base_type, B>> FullStateExpansion(
    const std::shared_ptr<CostExpansion<typename KP::base_type, B>> &E,
    const DiscreteDynamics<KP> *model) {
  return FullStateExpansion(statevectortype(model), E, model);
}
template <typename KP, bool B, typename SV = StateVectorType>
auto FullStateExpansion(SV type,
                        const CostExpansion<typename KP::base_type, B> &E,
                        const DiscreteDynamics<KP> *model) {
  CHECK(0);
}
template <typename KP, bool B>
std::shared_ptr<CostExpansion<typename KP::base_type, B>> FullStateExpansion(
    EuclideanState type,
    const std::shared_ptr<CostExpansion<typename KP::base_type, B>> &E,
    const DiscreteDynamics<KP> *model) {
  CHECK_EQ(errstate_dim(model), model->state_dim());
  return E;
}
template <typename KP, bool B>
std::shared_ptr<CostExpansion<typename KP::base_type, B>> FullStateExpansion(
    RotationState type,
    const std::shared_ptr<CostExpansion<typename KP::base_type, B>> &E,
    const DiscreteDynamics<KP> *model) {
  const int n = model->state_dim();
  const int m = model->control_dim();
  return std::make_shared<CostExpansion<typename KP::base_type, B>>(n, m,
                                                                    E.length());
}

template <typename O, typename C, typename P>
void cost_expansion(const O *obj, const C &E, const P &Z) {
  for (int k = 0; k < Z.size(); ++k) {
    gradient(default_diffmethod(obj), obj->cost_[k], E->at(k)->grad, Z[k]);
    hessian(default_diffmethod(obj), obj->cost_[k], E->at(k)->hess, Z[k]);
  }
}
template <typename M, typename C, typename P, typename Q>
void error_expansion(const std::vector<M> &models, C &Eerr, const C &Efull,
                     P &G, const Q &Z) {
  _error_expansion(statevectortype(models.front().get()), models, Eerr, Efull,
                   G, Z);
}
template <typename M, typename C, typename P, typename Q,
          typename SV = StateVectorType>
void _error_expansion(SV type, const std::vector<M> &models, C &Eerr,
                      const C &Efull, P &G, const Q &Z) {}
template <typename M, typename C, typename P, typename Q>
void _error_expansion(RotationState type, const std::vector<M> &models, C &Eerr,
                      const C &Efull, P &G, const Q &Z) {
  // TODO. Need to check equality.
  // CHECK(Eerr == Efull);
  const int N = length(Z);
  for (int k = 0; k < Eerr.size(); ++k) {
    const auto &model = models[std::min(k, N - 2)];
    _error_expansion(model, Eerr[k], Efull[k], G[k], G.back(), Z[k]);
  }
}
template <typename M, typename C, typename P, typename Q>
void _error_expansion(const M &model, C &E, const C &cost, const P &G, P &tmp,
                      const Q &z) {
  E->xx.setZero();
  E->uu = cost->uu;
  E->u = cost->u;
  // TODO. Impl this.
  // d_errstate_jacobian(model, E->xx, z.state(), cost->x);
  E->ux = cost->ux * G;
  E->x = G.adjoint() * cost->x;
  tmp = cost->xx * G;
  // TODO: Not sure if it's right!
  E->xx = G.adjoint() * tmp;
}

#endif
