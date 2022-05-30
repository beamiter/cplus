#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <type_traits>
#include <vector>
#include <memory>

#include <glog/logging.h>
using namespace google;

// #include "robot_dynamics/car_model.h"
// #include "robot_dynamics/discrete_dynamics.h"
// #include "robot_dynamics/dynamics.h"
// #include "robot_dynamics/functionbase.h"

using namespace std;

using Eigen::MatrixX;
using Eigen::MatrixXd;
using Eigen::VectorXd;

enum class TE : uint8_t {
  a = 0,
  b,
  c,
};

class A {
public:
  operator bool() {
    return true;
  }
  virtual TE get() const { return TE::a; }
};

class B : public A {
public:
  TE get() const override { return TE::b; }
};

class C : public A {
public:
  C() = default;
  C(int x0, int y0) {
    x = x0;
    y = y0;
  }
  void init(int x0, int y0) { *this = C(x0, y0); }
  TE get() const override { return TE::c; }
  int x = 0;
  int y = 0;
};

struct Pose2D {
  float x;
  float y;
  float yaw;

  Pose2D() : x(0), y(0), yaw(0) {}
  Pose2D(const Pose2D &pose) : x(pose.x), y(pose.y), yaw(pose.yaw) {}
  Pose2D(float x, float y, float yaw) : x(x), y(y), yaw(yaw) {}
  Pose2D operator=(const Pose2D &from) {
    Pose2D a(from.x, from.y, from.yaw);
    return a;
  }
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  std::shared_ptr<int> pp;
  if (pp) {
    LOG(INFO) << "HEHE;;";
  }
  A a;
  double aa = a;
  int aaa = a;
  float aaaa = a;
  if (a) {
    LOG(INFO) << "HEHE;;";
  }
  if (&a) {
    LOG(INFO) << "HEHE;;";
  }

  Pose2D p1(1, 2, 3);
  // Pose2D p(p1);
  Pose2D p2 = p1;
  LOG(INFO) << p1.x << ", " << p2.x;

  google::ShutdownGoogleLogging();
}
