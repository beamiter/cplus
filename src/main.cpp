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

using Eigen::MatrixX;
using Eigen::MatrixXd;
using Eigen::seq;
using Eigen::Vector;
using Eigen::VectorXd;

class A {
public:
  static constexpr int a = 12;
  void test() { LOG(INFO) << "HEHE;"; }
};
class B : public A {
public:
  void func() {
    LOG(INFO) << A::a;
    test();
  }
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  B b;
  b.func();

  google::ShutdownGoogleLogging();
}
