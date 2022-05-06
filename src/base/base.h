#ifndef BASE_H
#define BASE_H

#include <Eigen/Dense>
#include <vector>

using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T> auto length(T) { return -1; }
template <> inline auto length(VectorXd t) { return t.size(); }
template <> inline auto length<std::vector<double>>(std::vector<double> t) {
  return t.size();
}

template <typename T> auto zero(T) { return -1;}
template <> inline auto zero(double) { return 0.0;}
template <> inline auto zero(float) { return 0.0;}
template <> inline auto zero(int) { return 0;}

template <typename T> struct ValueType { typedef T value_type; };

template <typename T, typename U> struct is_same_type {
  const static bool value = false;
};

template <> struct is_same_type<VectorXd, VectorXd> {
  const static bool value = true;
};

template <> struct is_same_type<MatrixXd, MatrixXd> {
  const static bool value = true;
};

template <bool T> struct ValBool {};
template <int T> struct ValInt {};

#endif
