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

// template <typename T> using m_data_type = MatrixX<T>;
// template <typename T> using v_data_type = VectorX<T>;

// State and control
template <typename T, bool B = true> class StateControlExpansion {
public:
  static constexpr bool state_control = B;
  typedef MatrixX<T> m_data_type;
  typedef VectorX<T> v_data_type;

  StateControlExpansion(int n, int m)
      : ix(0, n), iu(n, m), data(MatrixX<T>::Zero(n + m, n + m + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), ux(data(iu, ix)), uu(data(iu, iu)), x(grad(ix)),
        u(grad(iu)) {
    assert(n > 0 && m > 0);
    std::cout << "Initialize finished\n";
  }

  Eigen::ArithmeticSequence<int, int> ix;
  Eigen::ArithmeticSequence<int, int> iu;
  m_data_type data;
  Ref<m_data_type> hess;
  Ref<v_data_type> grad;
  Ref<m_data_type> xx;
  Ref<m_data_type> ux;
  Ref<m_data_type> uu;
  Ref<v_data_type> x;
  Ref<v_data_type> u;
};
// State only
template <typename T> class StateControlExpansion<T, false> {
public:
  static const bool state_control = false;
  typedef MatrixX<T> m_data_type;
  typedef VectorX<T> v_data_type;

  StateControlExpansion(int n)
      : ix(0, n), data(MatrixX<T>::Zero(n, n + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)) {
    assert(n > 0);
    std::cout << "Specialization finished\n";
  }

  Eigen::ArithmeticSequence<int, int> ix;
  m_data_type data;
  Ref<m_data_type> hess;
  Ref<v_data_type> grad;
  Ref<m_data_type> xx;
  Ref<v_data_type> x;
};

template <typename T> struct StateControlExpansionHelper {
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

  CostExpansion() {}
  CostExpansion(const std::vector<int> &nx, const std::vector<int> &nu = {}) {
    const auto N = nx.size();
    const_hess.resize(N);
    const_grad.resize(N);

    const bool has_zero_control =
        nu.empty() || std::any_of(nu.begin(), nu.end(),
                                  [](const int &num) { return num == 0; });
    // Logitc violation
    if (has_zero_control && B == true) {
      assert(0);
    }
    if (has_zero_control) {
      for (auto k = 0; k < N; ++k) {
        data.push_back(StateControlExpansionHelper<T>(nx[k]));
      }
    } else {
      for (auto k = 0; k < N; ++k) {
        data.push_back(StateControlExpansionHelper<T>(nx[k], nu[k]));
      }
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
  static auto init(int n, int m, int N) {
    return CostExpansion<T, B>(std::vector<int>(N, n), std::vector<int>(N, m));
  }
};

template <typename T, bool B, AbstractModelTypeName>
auto FullStateExpansion(const CostExpansion<T, B> &E,
                        const DiscreteDynamicsDeclare &model) {
  return FullStateExpansion(statevectortype(model), E, model);
}
template <typename T, bool B, AbstractModelTypeName>
auto FullStateExpansion(EuclideanState, const CostExpansion<T, B> &E,
                        const DiscreteDynamicsDeclare &) {
  return E;
}
template <typename T, bool B, AbstractModelTypeName>
auto FullStateExpansion(RotationState, const CostExpansion<T, B> &E,
                        const DiscreteDynamicsDeclare &model) {
  const int n = state_dim(model);
  const int m = control_dim(model);
  return CostExpansionHelper<T, B>::init(n, m, E.length());
}

#endif
