#ifndef BASE_H
#define BASE_H

#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXcd;
using Eigen::VectorXd;
using Eigen::VectorXf;

enum class FunctionInputs {
  StateOnly,
  ControlOnly,
  StateControl,
};

struct FunctionSignature {};
struct Inplace : FunctionSignature {};
struct StaticReturn : FunctionSignature {};

struct StateVectorType {};
struct EuclideanState : StateVectorType {};
struct RotationState : StateVectorType {};

// enum class FunctionSignature {
// Inplace,
// StaticReturn,
//};

enum class DiffMethod {
  ForwardAD,
  FiniteDifference,
  UserDefined,
};

template <typename T> auto length(T) { return -1; }
template <> inline auto length(VectorXd t) { return t.size(); }
template <> inline auto length<std::vector<double>>(std::vector<double> t) {
  return t.size();
}

template <typename T> auto zero(T) { return -1; }
template <> inline auto zero(double) { return 0.0; }
template <> inline auto zero(float) { return 0.0; }
template <> inline auto zero(int) { return 0; }

template <typename T, typename U> struct is_same_type {
  const static bool value = false;
};
template <typename T> struct UseStatic { static constexpr bool val = false; };
template <> struct UseStatic<MatrixXd> { static constexpr bool val = true; };
template <> struct UseStatic<MatrixXf> { static constexpr bool val = true; };
template <> struct UseStatic<VectorXd> { static constexpr bool val = true; };
template <> struct UseStatic<VectorXf> { static constexpr bool val = true; };

template <> struct is_same_type<VectorXd, VectorXd> {
  const static bool value = true;
};
template <> struct is_same_type<MatrixXd, MatrixXd> {
  const static bool value = true;
};

template <bool T> struct ValBool {};
template <int T> struct ValInt {};

template <typename Func> void loop(int start, int end, Func f) {
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
