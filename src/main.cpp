#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <functional>
#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>
#include <atomic>

#include <glog/logging.h>
using namespace google;

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
  void test() const { LOG(INFO) << "HEHE;"; }
  virtual Sig default_sig() const { return Sig::none; }
  virtual Type default_type() { return Type(); }
};
class B : public A {
public:
  static constexpr int b = 17;
  void func() const {
    LOG(INFO) << A::a;
    test();
  }
  void fun() const {
    LOG(INFO) << "HAVE FUN";
  }
  Sig default_sig() const override { return Sig::inplace; }
  int data = 0;
  static const int goal = 15;
};
class C : public A {
public:
  Sig default_sig() const override { return Sig::st; }
};

template <typename T> void test(T) { LOG(INFO) << "default!"; }
template <> void test<A>(A) { LOG(INFO) << "A!"; }
template <> void test<B>(B) { LOG(INFO) << "B!"; }
template <> void test<C>(C) { LOG(INFO) << "C!"; }

template <Sig s> void test(Sig) { LOG(INFO) << "default!"; }
template <> void test<Sig::none>(Sig) { LOG(INFO) << "none!"; }
template <> void test<Sig::inplace>(Sig) { LOG(INFO) << "inplace!"; }
template <> void test<Sig::st>(Sig) { LOG(INFO) << "st!"; }

template<int T>
struct Val {};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  // A Dynamic_cast has runtime overhead because it checks object types at run
  // time using “Run-Time Type Information“.
  // If there is a surety we will never cast to wrong object then always avoid
  // dynamic_cast and use static_cast.
  B b;
  b.data = 13;
  LOG(INFO) << b.default_sig();
  const A *a = static_cast<const A *>(&b);
  LOG(INFO) << a->default_sig();
  const B *bb = static_cast<const B *>(a);
  LOG(INFO) << bb->default_sig() << ", " << bb->data << ", " << bb->goal;
  Val<B::goal>();
  bb->fun();

  std::atomic_flag lock = ATOMIC_FLAG_INIT;
  LOG(INFO) << lock.test_and_set();
  LOG(INFO) << lock.test_and_set();
  lock.clear();
  LOG(INFO) << lock.test_and_set();
  LOG(INFO) << lock.test_and_set();
  LOG(INFO) << lock.test_and_set();

  google::ShutdownGoogleLogging();
}
