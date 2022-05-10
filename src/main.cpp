#include <Eigen/Dense>
#include <iostream>
#include <type_traits>
#include <vector>

#include "robot_dynamics/car_model.h"
#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/dynamics.h"
#include "robot_dynamics/functionbase.h"

using namespace std;

struct A {};
struct AA : A {};
struct AAA : A {};

template <typename T> class B {
  static_assert(std::is_base_of<A, T>::value, "not derived from A");
  typedef typename std::enable_if<std::is_base_of<A, T>::value, T>::type base;

public:
  // virtual void test() = 0;
  void fund() { cout << ">>>>>>> basic function\n"; }
  void haha() {
    cout << "daidai\n";
  }
};

// template <> class B<A> {
// public:
// virtual void test() { cout << ">>>>>>> A\n"; }
//};
// template <> class B<AA> {
// public:
// virtual void test() { cout << ">>>>>>> AA\n"; }
//};
// template <> class B<AAA> {
// public:
// virtual void test() { cout << ">>>>>>> AAA\n"; }
//};

template <typename T> class C : public B<T> {
  void test() const { cout << ">>>>>>> c\n"; }
};

using Eigen::MatrixXd;

int main() {
  //B<int> ha;
  B<A> b;
  B<AA> b0;
  B<AAA> b1;
  b.haha();
  cout << std::is_base_of<A, AA>::value;

  cout << endl;
}
