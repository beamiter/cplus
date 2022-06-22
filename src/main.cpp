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

enum class Taste {
  good,
  bad,
};

struct A {
  constexpr Taste gettaste() { return Taste::bad; }
};

constexpr Taste gettaste(A) { return Taste::good; }

template <Taste t> void test() { LOG(INFO) << "default"; }
template <> void test<Taste::good>() { LOG(INFO) << "good"; }

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
  test<gettaste(a)>();
  test<a.gettaste()>();

  google::ShutdownGoogleLogging();
}
