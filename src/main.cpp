#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <type_traits>
#include <vector>

#include "robot_dynamics/car_model.h"
#include "robot_dynamics/discrete_dynamics.h"
#include "robot_dynamics/dynamics.h"
#include "robot_dynamics/functionbase.h"

using namespace std;

using Eigen::DiagonalMatrix;
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

template <TE T> void haha() { cout << "********** " << T << endl; }

int main() {
  haha<TE::b>();
  MatrixXd b;
  b.size();
}
