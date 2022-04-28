#ifndef ILQR_COST_EXPANTION
#define ILQR_COST_EXPANTION

#include <Eigen/Dense>

#include <iostream>
#include <vector>

#include "robot_dynamics/functionbase.h"
#include "robot_dynamics/statevectortype.h"

using Eigen::all;
using Eigen::last;
using Eigen::MatrixX;
using Eigen::Ref;
using Eigen::seq;
using Eigen::VectorX;

// State and control
template <typename T, bool B = true> struct StateControlExpansion {
  static const bool state_control = B;
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
template <typename T> struct StateControlExpansion<T, false> {
  static const bool state_control = false;
  typedef MatrixX<T> m_data_type;
  typedef VectorX<T> v_data_type;

  StateControlExpansion(int n, int m = 0)
      : ix(0, n), data(MatrixX<T>::Zero(n, n + 1)),
        hess(data(all, seq(0, last - 1))), grad(data(all, last)),
        xx(data(ix, ix)), x(grad(ix)) {
    assert(n > 0);
    if (m != 0) {
      std::cout << m << " Warning: \"m\" is not needed for state only!!!\n";
    }
    std::cout << "Specialization finished\n";
  }

  Eigen::ArithmeticSequence<int, int> ix;
  m_data_type data;
  Ref<m_data_type> hess;
  Ref<v_data_type> grad;
  Ref<m_data_type> xx;
  Ref<v_data_type> x;
};

template <typename T, bool B = true> struct CostExpansion {
  static const bool state_control = B;

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
        data.push_back(StateControlExpansion<T, B>(nx[k], 0));
      }
    } else {
      for (auto k = 0; k < N; ++k) {
        data.push_back(StateControlExpansion<T, B>(nx[k], nu[k]));
      }
    }
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

template <typename T, bool B, typename S> struct FullStateExpansion {};
template <typename T, bool B> struct FullStateExpansion<T, B, EuclideanState> {
  static auto value(const CostExpansion<T, B> &cost_expantion,
                    const int &model) {
    return cost_expantion;
  }
};

template <typename T, bool B> struct FullStateExpansion<T, B, RotationState> {
  static auto value(const CostExpansion<T, B> &cost_expantion,
                    const int &model) {
    const int n0 = 4;
    const int m = 2;
    return CostExpansionHelper<T, B>::init(n0, m, cost_expantion.length());
  }
};

#endif
