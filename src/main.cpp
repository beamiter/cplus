#include <Eigen/Dense>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/misc/lapacke.h>
#include <atomic> // std::atomic_flag
#include <chrono>
#include <functional>
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
  for (;;) {
    if (!lock_stream.test_and_set()) {
      cout << "do something \n";
    }
    lock_stream.clear();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
void append_number(int x) {
  for (;;) {
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

  std::shared_ptr<Eigen::MatrixXd> data = std::make_shared<Eigen::MatrixXd>(6, 8);
  Eigen::Ref<Eigen::MatrixXd> grad((*data)(Eigen::all, Eigen::last));
  data->setOnes();
  LOG(INFO) << *data;
  LOG(INFO) << grad;

  // std::vector<std::thread> threads;
  // threads.push_back(std::thread(do_something));
  // for (int i = 1; i <= 10; ++i)
  // threads.push_back(std::thread(append_number, i));
  // for (auto &th : threads)
  // th.join();

  Eigen::MatrixXd A(3, 3);
  A << 6, 0, 0, 0, 3, 0, 0, 0, 7;

  Eigen::MatrixXd L(A.llt().matrixL());
  Eigen::MatrixXd L_T = L.adjoint(); // conjugate transpose

  LOG(INFO) << "L" << std::endl;
  LOG(INFO) << L << std::endl;
  LOG(INFO) << "L_T" << std::endl;
  LOG(INFO) << L_T << std::endl;
  LOG(INFO) << "A" << std::endl;
  LOG(INFO) << A << std::endl;
  LOG(INFO) << "L*L_T" << std::endl;
  LOG(INFO) << L * L_T << std::endl;

  google::ShutdownGoogleLogging();
}
