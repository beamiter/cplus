#ifndef BASE_H
#define BASE_H

#include <Eigen/Dense>

using Eigen::VectorXd;

template <typename T> auto length(T) {}

template <> inline auto length(VectorXd t) { return t.size(); }

template <typename T> struct DataType { typedef T datatype; };

#endif
