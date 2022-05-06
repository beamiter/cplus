#include <gtest/gtest.h>

#include "ilqr/cost_expansion.h"
#include "ilqr/ilqr_solver.h"
#include "robot_dynamics/car_model.h"
#include "robot_dynamics/knotpoint.h"
#include "solver_opts.h"
#include "trajectory_optimization/problem.h"

//#include <glog/logging.h>

TEST(CostExpansionTest, StateControl) {
  auto sce1 = StateControlExpansion<double>(6, 2);
  sce1.grad(3) = 3;
  std::cout << sce1.data << std::endl;

  auto sce2 = StateControlExpansion<double, true>(6, 2);
  sce2.grad(3) = 4;
  std::cout << sce2.data << std::endl;

  auto sce3 = StateControlExpansion<double, false>(6, 2);
  sce3.grad(3) = 5;
  std::cout << sce3.data << std::endl;

  auto sce4 = StateControlExpansion<double, false>(6);
  sce4.grad(3) = 6;
  std::cout << sce4.data << std::endl;
  std::cout << sce4.state_control << std::endl;

  auto cost1 = CostExpansion<double>({1, 2, 3}, {1, 2, 3});
  auto cost2 = CostExpansion<double, false>({1, 2, 3}, {1, 2, 3});
  auto cost3 = CostExpansion<double, false>({1, 2, 3}, {0, 2, 3});
  auto cost4 = CostExpansion<double, false>({1, 2, 3});
  auto cost5 = CostExpansionHelper<double>::init(6, 2, 3);

  int model = 1;
  auto fse1 =
      FullStateExpansion<double, cost1.state_control, EuclideanState>::value(
          cost1, model);
  auto fse2 =
      FullStateExpansion<double, cost2.state_control, EuclideanState>::value(
          cost2, model);
  auto fse3 =
      FullStateExpansion<double, cost3.state_control, EuclideanState>::value(
          cost3, model);
  auto fse4 =
      FullStateExpansion<double, cost4.state_control, EuclideanState>::value(
          cost4, model);
  std::cout << fse4.data[0].data << std::endl << std::endl;
  auto fse5 =
      FullStateExpansion<double, cost5.state_control, RotationState>::value(
          cost5, model);
  std::cout << fse5[0].data << std::endl << std::endl;

  std::vector<int> ha({2,3,4});
  ha.insert(ha.begin(), 6);
  std::partial_sum(ha.begin(), ha.end(), ha.begin());
  for (auto a : ha) {
    std::cout << a << " -------------\n";
  }

  auto car = CarModel();
  Objective<double> obj;
  std::vector<double> x0({0, 0, 0, 0, 0, 0});
  double tf = 5.0;
  auto prob = ProblemHelper::init<6, 2>(car, obj, x0, tf);
  auto opts = SolverOptions<double>();
  auto stats = SolverStats<double>();
  // auto solver =
  //     iLQRSolverHelper::init(prob, opts, stats, ValBool<true>(), UserDefined());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  // google::InitGoogleLogging(argv[0]);

  std::cout << RUN_ALL_TESTS();

  // google::ShutdownGoogleLogging();
  return 1;
}
