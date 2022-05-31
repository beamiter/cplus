#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>

#include <glog/logging.h>
using namespace google;

// #include "robot_dynamics/car_model.h"
// #include "robot_dynamics/discrete_dynamics.h"
// #include "robot_dynamics/dynamics.h"
// #include "robot_dynamics/functionbase.h"

using namespace std;

using Eigen::MatrixX;
using Eigen::MatrixXd;
using Eigen::seq;
using Eigen::Vector;
using Eigen::VectorXd;

template <template <typename, typename> class V, typename T, typename P>
struct A {
  static constexpr int flag = 6;
  typedef int value_type;
  typedef float value_type1;
  typedef double value_type2;
  using VV = V<T, P>;
  VV a;
};
template <typename T> struct B {
  typename T::value_type1 b1;
  typename T::value_type2 b2;
  int N = T::flag;
};

Eigen::Ref<Vector<double, 5>> test(Vector<double, 8> &in) {
  return in(seq(0, 4));
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  Vector<double, 8> in;
  in << 1,2,3,4,5,6,7,8;
  auto rtn = test(in);
  rtn(3) = 190;
  Vector<double, 5> other = rtn;
  other(3) = 210;
  LOG(INFO) << "************ " << rtn;
  LOG(INFO) << "************ " << in;
  LOG(INFO) << "************ " << other;

  A<std::pair, int, int> tmp;
  A<std::pair, int, int>::value_type b = 8;
  tmp.a.first = 10;
  LOG(INFO) << tmp.a.first << ", " << b;

  B<A<std::pair, int, int>> btmp;
  LOG(INFO) << btmp.b1 << ", " << btmp.b2 << ", " << btmp.N;

  Vector<double, 4> d4, d5;
  d4 << 1, 2, 3, 4;
  VectorXd xd = d4;
  // xd << 5;
  d5 = xd;
  LOG(INFO) << xd;
  LOG(INFO) << d5;
  LOG(INFO) << std::is_base_of<VectorXd, Vector<double, 4>>::value;
  LOG(INFO) << std::is_base_of<Vector<double, 4>, Vector<double, 4>>::value;
  LOG(INFO) << std::is_base_of<Vector<double, 3>, Vector<double, 4>>::value;
  LOG(INFO) << std::is_base_of<VectorXd, VectorXd>::value;

  google::ShutdownGoogleLogging();
}
