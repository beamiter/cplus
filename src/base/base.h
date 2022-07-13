#ifndef BASE_H
#define BASE_H

#include <Eigen/Dense>
#include <exception>
#include <glog/logging.h>
#include <vector>

using namespace google;

using Eigen::Matrix;
using Eigen::MatrixX;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Vector;
using Eigen::VectorX;
using Eigen::VectorXcd;
using Eigen::VectorXd;
using Eigen::VectorXf;

#define DERIVE(Base, Derived)                                                  \
  struct Derived : Base {};

#define DECLARE(Base)                                                          \
  struct Base {};

DECLARE(FunctionInputs);
DERIVE(FunctionInputs, StateOnly);
DERIVE(FunctionInputs, ControlOnly);
DERIVE(FunctionInputs, StateControl);

DECLARE(StateVectorType);
DERIVE(StateVectorType, EuclideanState);
DERIVE(StateVectorType, RotationState);

DECLARE(FunctionSignature);
DERIVE(FunctionSignature, Inplace);
DERIVE(FunctionSignature, StaticReturn);

DECLARE(DiffMethod);
DERIVE(DiffMethod, ForwardAD);
DERIVE(DiffMethod, FiniteDifference);
DERIVE(DiffMethod, UserDefined);

// Deprecated.
template <typename T> int length(const T &) {
  CHECK(0);
  return -1;
}
template <typename T> int length(const std::vector<T> &t) { return t.size(); }
template <typename T> int length(const MatrixX<T> &t) { return t.size(); }
template <typename T> int length(const VectorX<T> &t) { return t.size(); }
template <typename T> int length(Eigen::Ref<MatrixX<T>> t) { return t.size(); }
template <typename T> int length(Eigen::Ref<VectorX<T>> t) { return t.size(); }
template <typename T, int N> auto length(const Vector<T, N> &t) { return N; }
template <typename T, int N, int M> auto length(const Matrix<T, N, M> &t) {
  return N * M;
}

template <typename T> auto zero(T) { return -1; }
template <> inline auto zero(double) { return 0.0; }
template <> inline auto zero(float) { return 0.0; }
template <> inline auto zero(int) { return 0; }

template <typename T, typename U> struct is_same_type {
  constexpr static bool value = false;
};
template <typename T> struct is_same_type<T, T> {
  constexpr static bool value = true;
};
template <typename T> struct is_same_type<std::vector<T>, std::vector<T>> {
  constexpr static bool value = true;
};
template <typename T> struct is_same_type<VectorX<T>, VectorX<T>> {
  constexpr static bool value = true;
};
template <typename T> struct is_same_type<MatrixX<T>, MatrixX<T>> {
  constexpr static bool value = true;
};

template <typename T> struct UseStatic { static constexpr bool value = true; };
template <typename T> struct UseStatic<std::vector<T>> {
  static constexpr bool value = false;
};
template <> struct UseStatic<MatrixXd> { static constexpr bool value = false; };
template <> struct UseStatic<MatrixXf> { static constexpr bool value = false; };
template <> struct UseStatic<VectorXd> { static constexpr bool value = false; };
template <> struct UseStatic<VectorXf> { static constexpr bool value = false; };

#define VAL(type)                                                              \
  template <type T> struct Val##type {};
VAL(bool);
VAL(int);

template <typename Func> void loop(int start, int end, const Func &f) {
  for (int k = start; k < end; ++k) {
    f(k);
  }
}

inline bool ispossemedef(MatrixXd A) {
  VectorXcd eigs = A.eigenvalues();
  return std::any_of(eigs.real().begin(), eigs.real().end(),
                     [](const auto val) { return val < 0; });
}

// https://stackoverflow.com/questions/35227131/eigen-check-if-matrix-is-positive-semi-definite
template <class MatrixT> bool isPsd(const MatrixT &A) {
  if (!A.isApprox(A.transpose())) {
    return false;
  }
  const auto ldlt = A.template selfadjointView<Eigen::Upper>().ldlt();
  if (ldlt.info() == Eigen::NumericalIssue) {
    return false;
  }
  return ldlt.isPositive();
}
template <class MatrixT> bool isPd(const MatrixT &A) {
  if (!A.isApprox(A.transpose())) {
    return false;
  }
  const auto ldlt = A.template selfadjointView<Eigen::Upper>().ldlt();
  if (ldlt.info() == Eigen::NumericalIssue) {
    return false;
  }
  return !ldlt.isNegative();
}

#endif
