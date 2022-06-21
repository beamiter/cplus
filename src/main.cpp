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

struct A {
  void test() {
    LOG(INFO) << "In A!";
  }
};
struct B : A {
  int data;
};
// void test(const B &) { LOG(INFO) << "not default!"; }
// void funny(const B &b) {
//   LOG(INFO) << "funny!";
//   test(b);
// }
void test(const A &a) { LOG(INFO) << "default!"; }
void fun(const A &a) {
  LOG(INFO) << "fun!";
  test(a);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  // A Dynamic_cast has runtime overhead because it checks object types at run
  // time using “Run-Time Type Information“.
  // If there is a surety we will never cast to wrong object then always avoid
  // dynamic_cast and use static_cast.
  std::atomic_flag lock = ATOMIC_FLAG_INIT;
  LOG(INFO) << lock.test_and_set();
  LOG(INFO) << lock.test_and_set();
  lock.clear();
  LOG(INFO) << lock.test_and_set();

  B b;
  test(b);
  fun(b);

  google::ShutdownGoogleLogging();
}
