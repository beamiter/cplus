#include "base/base.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <atomic>
#include <atomic> // std::atomic_flag
#include <chrono>
#include <functional>
#include <iostream>
#include <iostream> // std::cout
#include <memory>
#include <sstream> // std::stringstream
#include <thread>  // std::thread
#include <type_traits>
#include <vector> // std::vector

#include <glog/logging.h>
using namespace google;

using namespace std;

std::atomic_flag lock_stream = ATOMIC_FLAG_INIT;

void do_something() {
  while (1) {
    if (!lock_stream.test_and_set()) {
      cout << "do something \n";
    }
    lock_stream.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
void append_number(int x) {
  while (1) {
    while (lock_stream.test_and_set()) {
    }
    cout << "thread #" << x << '\n';
    lock_stream.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

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

  std::vector<std::thread> threads;
  threads.push_back(std::thread(do_something));
  for (int i = 1; i <= 10; ++i)
    threads.push_back(std::thread(append_number, i));
  for (auto &th : threads)
    th.join();

  return 0;

  google::ShutdownGoogleLogging();
}
