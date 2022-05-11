#ifndef ILQR_COST_EXPANTION
#define ILQR_COST_EXPANTION

#include <Eigen/Dense>

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
template <typename T, bool B = true> class StateControlExpansion {
public:
  static constexpr bool state_control = B;
  typedef MatrixX<T> m_data_type;
  typedef VectorX<T> v_data_type;

#if B
  StateControlExpansion(int n, int m)
      : ix(0, n), data(MatrixX<T>::Zero(n + m, n + m + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)), iu(n, m), ux(data(iu, ix)),
        uu(data(iu, iu)), u(grad(iu)) {
    assert(n > 0 && m > 0);
    std::cout << "Initialize finished\n";
  }
#else
  // m is not essential
  StateControlExpansion(int n, int m = 0)
      : ix(0, n), data(MatrixX<T>::Zero(n, n + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)) {
    assert(n > 0);
    std::cout << "Specialization finished\n";
  }
#endif

  Eigen::ArithmeticSequence<int, int> ix;
  m_data_type data;
  Ref<m_data_type> hess;
  Ref<v_data_type> grad;
  Ref<m_data_type> xx;
  Ref<v_data_type> x;
#if B
  Eigen::ArithmeticSequence<int, int> iu;
  Ref<m_data_type> ux;
  Ref<m_data_type> uu;
  Ref<v_data_type> u;
#endif
};

template <typename T> class StateControlExpansionHelper {
public:
  StateControlExpansion<T, true> operator()(int n, int m) {
    return StateControlExpansion<T, true>(n, m);
  }
  StateControlExpansion<T, false> operator()(int n) {
    return StateControlExpansion<T, false>(n);
  }
};

template <typename T, bool B = true> class CostExpansion {
public:
  static const bool state_control = B;

  CostExpansion() = default;
  CostExpansion(const std::vector<int> &nx, const std::vector<int> &nu = {}) {
    const auto N = nx.size();
    const_hess.resize(N);
    const_grad.resize(N);
    for (auto k = 0; k < N; ++k) {
      data.push_back(StateControlExpansion<T, B>(nx[k], nu[k]));
    }
  }

  void operator()(const std::vector<int> &nx, const std::vector<int> &nu = {}) {
    CostExpansion(nx, nu);
  }

  const auto &operator[](int i) const { return data[i]; }
  int size() const { return data.size(); }
  int length() const { return data.size(); }

  std::vector<StateControlExpansion<T, B>> data;
  std::vector<bool> const_hess;
  std::vector<bool> const_grad;
};

template <typename T, bool B = true> struct CostExpansionHelper {
  CostExpansion<T, B> operator()(int n, int m, int N) {
    return CostExpansion<T, B>(std::vector<int>(N, n), std::vector<int>(N, m));
  }
};

template <typename T, bool B, AbstractModelTypeName>
CostExpansion<T, B> FullStateExpansion(const CostExpansion<T, B> &E,
                                       const DiscreteDynamicsDeclare *model) {
  return FullStateExpansion(typename AbstractModelDeclare::statevectortype(), E,
                            model);
  return FullStateExpansion(S(), E, model);
}
template <typename T, bool B, AbstractModelTypeName>
CostExpansion<T, B> FullStateExpansion(EuclideanState,
                                       const CostExpansion<T, B> &E,
                                       const DiscreteDynamicsDeclare *) {
  return E;
}
template <typename T, bool B, AbstractModelTypeName>
CostExpansion<T, B> FullStateExpansion(RotationState,
                                       const CostExpansion<T, B> &E,
                                       const DiscreteDynamicsDeclare *model) {
  const int n = model->state_dim();
  const int m = model->control_dim();
  return CostExpansionHelper<T, B>::init(n, m, E.length());
}

#endif
