#ifndef BASE_H
#define BASE_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T> auto length(T) { return -1; }

template <> inline auto length(VectorXd t) { return t.size(); }

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

template <typename T> struct Val {
  T data;
};
template <> 
struct Val<bool> {
  bool data;
  Val(bool b) {
    data = b;
  }
  bool operator==(const Val<bool> rhs) {
    return this->data == rhs.data;
  }
};

#endif
