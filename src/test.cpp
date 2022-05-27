#include <glog/logging.h>
#include <gtest/gtest.h>

#include "ilqr/cost_expansion.h"
#include "ilqr/ilqr_solver.h"
#include "robot_dynamics/car_model.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "solver_opts.h"
#include "trajectory_optimization/problem.h"

using namespace google;

TEST(CostExpansionTest, StateControl) {
  auto de = DynamicsExpansion<double>::init(6, 7, 2);
  de.A(2, 2) = 56;
  de.A(4, 3) = 56;
  LOG(INFO) << de.A.size();
  LOG(INFO) << de.A;
  de.B(0, 1) = 156;
  LOG(INFO) << de.B;
  LOG(INFO) << de.B.size();
  LOG(INFO) << de.Df;

  MatrixXd d(3, 4);
  d.setRandom();
  LOG(INFO) << d << std::endl;
  d.setZero();
  LOG(INFO) << d << std::endl;
  d.diagonal().setOnes();
  LOG(INFO) << d << std::endl;
  d.setOnes();
  LOG(INFO) << d << std::endl;
  auto dd = d(all, 1);
  dd(0) = 10;
  dd(1) = 8;
  dd(2) = 6;
  LOG(INFO) << d << std::endl;

  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  auto traj = SampledTrajectoryX<6, 2, double>();
  traj.data.push_back(KnotPointX<6, 2, double>());
  std::vector<double> x0({0, 0, 0, 0, 0, 0});
  std::vector<double> xf({12, 0, 0, 0, 0, 0});
  std::vector<double> uf({0, 0});
  double tf = 5.0;
  auto prob = CarProblem<6, 2, double>(x0, xf, uf, 51, 0.1);
  LOG(INFO) << prob.N;
  LOG(INFO) << prob.car_.state_dim();
  LOG(INFO) << prob.model.size();
  LOG(INFO) << prob.model.front()->state_dim();
  auto solver = iLQRSolver<6, 6, 2, double, VectorXd>(
      &prob, opts, stats, DiffMethod::UserDefined, ValBool<true>(),
      ValInt<6>());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  // Set logging level
  google::SetStderrLogging(google::GLOG_INFO);

  LOG(INFO) << RUN_ALL_TESTS();

  google::ShutdownGoogleLogging();
  return 1;
}
