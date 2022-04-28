#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "ilqr/cost_expansion.h"

using Eigen::MatrixXd;

int main() {
  MatrixXd m(2, 2);
  m(0, 0) = 1;
  m(0, 1) = 2;
  m(1, 0) = 3;
  m(1, 1) = 4;
  std::cout << m << std::endl;
  std::cout << "great\n";
}
