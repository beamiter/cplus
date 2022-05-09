#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "robot_dynamics/functionbase.h"
// #include "robot_dynamics/dynamics.h"

using namespace std;

struct A {
  virtual void test() {
    cout << "a\n";
  }
};
struct B:A {
  // virtual void test() {
  //   cout << "b\n";
  // }
};

struct C:B {
};

using Eigen::MatrixXd;

int main() {
  C c;
  c.test();
  std::cout << "great\n";
  KnotPointXd<3, 5> kp;
}
