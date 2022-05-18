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

using Eigen::MatrixXd;
using Eigen::DiagonalMatrix;
using Eigen::VectorXd;

int main() {
  Eigen::Vector4d a(4,6,5,7);
  Eigen::Vector<double, 5> b(4,6,7,5, 9);
  cout << b << endl;
  DiagonalMatrix<double, 5> c(b);
  cout << c.diagonal() << endl;
}
