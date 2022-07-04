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
  StateControlExpansion(int n, int m = -1)
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
    const_hess.resize(N, false);
    const_grad.resize(N, false);
    for (auto k = 0; k < N; ++k) {
      data.push_back(
          std::make_shared<StateControlExpansion<T, B>>(nx[k], nu[k]));
    }
  }
  CostExpansion(int n, int m, int N) {
    const_hess.resize(N, false);
    const_grad.resize(N, false);
    for (auto k = 0; k < N; ++k) {
      data.push_back(std::make_shared<StateControlExpansion<T, B>>(n, m));
    }
  }
  CostExpansion(const CostExpansion &in)
      : data(in.data), const_hess(in.const_hess), const_grad(in.const_grad) {}

  const auto &operator[](int i) const { return data[i]; }
  auto &operator[](int i) { return data[i]; }
  int size() const { return data.size(); }
  int length() const { return data.size(); }

  std::vector<std::shared_ptr<StateControlExpansion<T, B>>> data;
  std::vector<bool> const_hess;
  std::vector<bool> const_grad;
};

template <typename KP, bool B>
CostExpansion<typename KP::base_type, B>
FullStateExpansion(const CostExpansion<typename KP::base_type, B> &E,
                   const DiscreteDynamics<KP> *model) {
  return FullStateExpansion(model->statevectortype(), E, model);
}
template <typename KP, bool B>
CostExpansion<typename KP::base_type, B>
FullStateExpansion(StateVectorType type,
                   const CostExpansion<typename KP::base_type, B> &E,
                   const DiscreteDynamics<KP> *model) {
  if (type == StateVectorType::EuclideanState) {
    return E;
  } else {
    const int n = model->state_dim();
    const int m = model->control_dim();
    return CostExpansion<typename KP::base_type, B>(n, m, E.length());
  }
}

template <typename O, typename C, typename P>
void cost_expansion(const O *obj, C &E, const P &Z) {
  for (int k = 0; k < Z.size(); ++k) {
    gradient(obj->diff_method[k], obj->cost_[k], E[k]->grad, Z[k]);
    hessian(obj->diff_method[k], obj->cost_[k], E[k]->hess, Z[k]);
  }
}
template <typename M, typename C, typename P, typename Q>
void error_expansion(const std::vector<M> &models, C Eerr, C Efull, P G, Q Z) {
  _error_expansion(models.front()->statevectortype(), models, Eerr, Efull, G,
                   Z);
}
template <typename M, typename C, typename P, typename Q>
void _error_expansion(StateVectorType type, const std::vector<M> &models,
                      C Eerr, C Efull, P G, Q Z) {
  CHECK(0);
  // TODO. Need to check equality.
  // CHECK(Eerr == Efull);
  if (StateVectorType::EuclideanState == type) {
    return;
  } else if (StateVectorType::RotationState == type) {
    const int N = length(Z);
    for (int k = 0; k < Eerr.size(); ++k) {
      const auto &model = models[std::min(k, N - 2)];
      _error_expansion(model, Eerr[k], Efull[k], G[k], G.back(), Z[k]);
    }
  }
}
template <typename M, typename C, typename P, typename Q>
void _error_expansion(const M &model, C E, C cost, P G, P tmp, Q z) {
  E.xx = 0;
  E.uu = cost.uu;
  E.u = cost.u;
  d_errstate_jacobian(model, E.xx, z.state(), cost.x);
  E.ux = cost.ux * G;
  E.x = G.adjoint() * cost.x;
  tmp = cost.xx * G;
  // TODO: Not sure if it's right!
  // E.xx = G.adjoint() * tmp;
}

#endif
