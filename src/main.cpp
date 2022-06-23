#include "base/base.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>

#include <glog/logging.h>
using namespace google;

using namespace std;

struct Taste {};
struct Good : Taste {};
struct Bad : Taste {};

struct A {
  constexpr Good gettaste() { return Good(); }
};

constexpr Bad gettaste(A) { return Bad(); }

template <typename T = Taste> void test(T) { LOG(INFO) << "default"; }
template <> void test(Good) { LOG(INFO) << "good"; }
template <> void test(Bad) { LOG(INFO) << "bad"; }

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  std::atomic_flag lock = ATOMIC_FLAG_INIT;
  LOG(INFO) << lock.test_and_set();
  LOG(INFO) << lock.test_and_set();
  lock.clear();
  LOG(INFO) << lock.test_and_set();

  A a;
  test(1);
  test(a.gettaste());
  test(gettaste(a));

  LOG(INFO) << std::is_base_of<Eigen::DiagonalMatrix<double, -1>,
                               Eigen::DiagonalMatrix<double, 5>>::value;
  LOG(INFO) << std::is_base_of<Eigen::DiagonalMatrix<double, -1>,
                               Eigen::DiagonalMatrix<double, -1>>::value;
  LOG(INFO) << is_same_type<Eigen::DiagonalMatrix<double, -1>,
                            Eigen::DiagonalMatrix<double, 5>>::value;
  LOG(INFO) << is_same<Eigen::DiagonalMatrix<double, -1>,
                       Eigen::DiagonalMatrix<double, 5>>::value;
  LOG(INFO) << is_same_type<Eigen::DiagonalMatrix<double, -1>,
                            Eigen::MatrixXd>::value;

  google::ShutdownGoogleLogging();
}
