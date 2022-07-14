#include "base/base.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <type_traits>
#include <vector>

#include <glog/logging.h>
using namespace google;

using namespace std;

struct Knot {
  Knot() : state(data(Eigen::seq(0, 3), Eigen::seq(0, 3))) {}
  Eigen::Matrix<double, 6, 8> data;
  Eigen::Ref<Eigen::MatrixX<double>> state;
  Eigen::Matrix<double, 3, 3> getstate() const { return state; }
  Eigen::Ref<Eigen::MatrixX<double>> getstate() { return state; }
  Eigen::Ref<Eigen::MatrixX<double>> getdata() { return data; }
};

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  std::shared_ptr<MatrixXd> data = std::make_shared<MatrixXd>(6, 8);
  Eigen::Ref<MatrixXd> grad((*data)(Eigen::all, Eigen::last));
  data->setOnes();
  LOG(INFO) << *data;
  LOG(INFO) << grad;

  google::ShutdownGoogleLogging();
}
