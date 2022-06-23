#ifndef ILQR_COST_EXPANTION
#define ILQR_COST_EXPANTION

#include <Eigen/Dense>

#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <vector>

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
  typedef MatrixX<T> m_data_type;
  typedef VectorX<T> v_data_type;
  static_assert(state_control, "For state and control!!!");

  StateControlExpansion(int n, int m)
      : ix(0, n), data(MatrixX<T>::Zero(n + m, n + m + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)), iu(n, m), ux(data(iu, ix)),
        uu(data(iu, iu)), u(grad(iu)) {
    hess.setIdentity();
    assert(n > 0 && m > 0);
  }

  Eigen::ArithmeticSequence<int, int> ix;
  m_data_type data;
  Ref<m_data_type> hess;
  Ref<v_data_type> grad;
  Ref<m_data_type> xx;
  Ref<v_data_type> x;
  Eigen::ArithmeticSequence<int, int> iu;
  Ref<m_data_type> ux;
  Ref<m_data_type> uu;
  Ref<v_data_type> u;
};
template <typename T> class StateControlExpansion<T, false> {
public:
  static constexpr bool state_control = false;
  typedef MatrixX<T> m_data_type;
  typedef VectorX<T> v_data_type;
  static_assert(!state_control, "For only state!!!");

  // m is not essential
  StateControlExpansion(int n, int m = -1)
      : ix(0, n), data(MatrixX<T>::Zero(n, n + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)) {
    hess.setIdentity();
    assert(n > 0);
  }

  Eigen::ArithmeticSequence<int, int> ix;
  m_data_type data;
  Ref<m_data_type> hess;
  Ref<v_data_type> grad;
  Ref<m_data_type> xx;
  Ref<v_data_type> x;
};

template <typename T, bool B = true> class CostExpansion {
public:
  static const bool state_control = B;

  CostExpansion(const std::vector<int> &nx, const std::vector<int> &nu = {}) {
    const auto N = nx.size();
    const_hess.resize(N, false);
    const_grad.resize(N, false);
    for (auto k = 0; k < N; ++k) {
      data.push_back(StateControlExpansion<T, B>(nx[k], nu[k]));
    }
  }
  // Delegating constructors
  CostExpansion(int n, int m, int N)
      : CostExpansion(std::vector<int>(N, n), std::vector<int>(N, m)) {}
  CostExpansion(const CostExpansion &in)
      : data(in.data), const_hess(in.const_hess), const_grad(in.const_grad) {}

  const auto &operator[](int i) const { return data[i]; }
  int size() const { return data.size(); }
  int length() const { return data.size(); }

  std::vector<StateControlExpansion<T, B>> data;
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

#endif
