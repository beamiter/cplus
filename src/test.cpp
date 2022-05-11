#include <gtest/gtest.h>

#include "ilqr/cost_expansion.h"
#include "ilqr/ilqr_solver.h"
#include "robot_dynamics/car_model.h"
#include "robot_dynamics/knotpoint.h"
#include "robot_dynamics/trajectories.h"
#include "solver_opts.h"
#include "trajectory_optimization/problem.h"

//#include <glog/logging.h>

TEST(CostExpansionTest, StateControl) {
  auto de = DynamicsExpansion<double>::init(6, 7, 2);
  de.A(2, 2) = 56;
  de.A(4, 3) = 56;
  std::cout << de.A.size() << std::endl;
  std::cout << de.A << std::endl;
  de.B(0, 1) = 156;
  std::cout << de.B << std::endl;
  std::cout << de.B.size() << std::endl;
  std::cout << de.Df << std::endl;

  MatrixXd d(3, 4);
  d.setRandom();
  std::cout << d << std::endl;
  d.setZero();
  std::cout << d << std::endl;
  d.diagonal().setOnes();
  std::cout << d << std::endl;
  d.setOnes();
  std::cout << d << std::endl;
  auto dd = d(all, 1);
  dd(0) = 10;
  dd(1) = 8;
  dd(2) = 6;
  std::cout << d << std::endl;

  auto car = CarModel<Inplace, EuclideanState>();
  Objective<double> obj;
  std::vector<double> x0({0, 0, 0, 0, 0, 0});
  double tf = 5.0;
  auto prob = ProblemHelper::init<6, 2>(car, obj, x0, tf);
  auto opts = SolverOptionsD();
  auto stats = SolverStatsD();
  auto traj = SampledTrajectoryX<6, 2, double>();
  traj.data.push_back(KnotPointX<6, 2, double>());
  std::cout << "****************" << std::endl;
  // auto solver = iLQRSolver<6, 7, 2, double, VectorXd, Inplace,
  // EuclideanState>( prob, opts, stats, DiffMethod::UserDefined,
  // ValBool<true>(), ValInt<7>());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // google::InitGoogleLogging(argv[0]);

  std::cout << RUN_ALL_TESTS();

  // google::ShutdownGoogleLogging();
  return 1;
}
