#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <functional>
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

enum Sig {
  none,
  inplace,
  st,
};
struct Type {};
struct Eucl : Type {};
struct Rote : Type {};

class A {
public:
  static constexpr int a = 12;
  void test() { LOG(INFO) << "HEHE;"; }
  virtual Sig default_sig() { return Sig::none; }
  virtual Type default_type() { return Type(); }
};
class B : public A {
public:
  void func() {
    LOG(INFO) << A::a;
    test();
  }
  Sig default_sig() override { return Sig::inplace; }
};
class C : public A {
public:
  Sig default_sig() override { return Sig::st; }
};

template <typename T> void test(T) { LOG(INFO) << "default!"; }
template <> void test<A>(A) { LOG(INFO) << "A!"; }
template <> void test<B>(B) { LOG(INFO) << "B!"; }
template <> void test<C>(C) { LOG(INFO) << "C!"; }

template <Sig s> void test(Sig) { LOG(INFO) << "default!"; }
template <> void test<Sig::none>(Sig) { LOG(INFO) << "none!"; }
template <> void test<Sig::inplace>(Sig) { LOG(INFO) << "inplace!"; }
template <> void test<Sig::st>(Sig) { LOG(INFO) << "st!"; }

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  // A Dynamic_cast has runtime overhead because it checks object types at run
  // time using “Run-Time Type Information“.
  // If there is a surety we will never cast to wrong object then always avoid
  // dynamic_cast and use static_cast.
  B b;
  LOG(INFO) << b.default_sig();
  A *a = static_cast<A *>(&b);
  LOG(INFO) << a->default_sig();
  B *bb = static_cast<B *>(a);
  LOG(INFO) << bb->default_sig();

  google::ShutdownGoogleLogging();
}
