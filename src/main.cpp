#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <type_traits>
#include <vector>

#include <glog/logging.h>
using namespace google;

// #include "robot_dynamics/car_model.h"
// #include "robot_dynamics/discrete_dynamics.h"
// #include "robot_dynamics/dynamics.h"
// #include "robot_dynamics/functionbase.h"

using namespace std;

using Eigen::DiagonalMatrix;
using Eigen::MatrixX;
using Eigen::MatrixXd;
using Eigen::VectorXd;

enum TE {
  a,
  b,
  c,
};

class A {
public:
  virtual TE get() const = 0;
};

class B : public A {
public:
  TE get() const override { return TE::b; }
};

class C : public A {
public:
  TE get() const override { return TE::c; }
};

template <TE T> void haha() { LOG(INFO) << "********** " << T << endl; }

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);

  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  MatrixXd f(3, 3);
  VectorXd b(3), u(3);
  b << 1, 2, 3;
  u << 4, 5, 6;
  f << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  double cf = 0.5 * b.adjoint() * f * b;
  cf += 0.5 * u.adjoint() * u;
  LOG(INFO) << cf << endl;
  LOG(INFO) << f.size();
  LOG(INFO) << f.rows() << f.cols();

  google::ShutdownGoogleLogging();
}
