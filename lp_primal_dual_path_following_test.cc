#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "lp.h"
#include "parser.h"

TEST(LPModel, PrimalDualPathFollowingSolve1) {
  Parser parser;
  std::ifstream file("tests/test3.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();

  auto result = model.PrimalDualPathFollowingSolve(1e-9, {});

  EXPECT_EQ(result, SOLVED);
  std::cout << model.GetPrimalDualPathFollowingOptimum().ToString() << "\n";

  EXPECT_LE(-7.0f - model.GetPrimalDualPathFollowingOptimum(), 1e-3f);
  EXPECT_GE(-7.0f - model.GetPrimalDualPathFollowingOptimum(), -1e-3f);

  auto expected_sol = std::map<Variable, Num>({{x1, 5.0f}, {x2, 2.0f}});

  auto actual_sol = model.GetPrimalDualPathFollowingSolution();

  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(entry.second - actual_sol[entry.first], 1e-3f);
    EXPECT_GE(entry.second - actual_sol[entry.first], -1e-3f);
  }
}

// TEST(LPModel, PrimalDualPathFollowingSolve2) {
//   Parser parser;
//   std::ifstream file("tests/test4.txt");
//   LPModel model = parser.Parse(file);
//   Variable x1("x1"), x2("x2");
//   model.ToStandardForm();
//   model.ToSlackForm();

//   auto result = model.PrimalDualPathFollowingSolve(1e-9, {});

//   EXPECT_EQ(result, SOLVED);
//   std::cout << model.GetPrimalDualPathFollowingOptimum().ToString() << "\n";

//   auto expected_sol = std::map<Variable, Num>({{x1, 2.0 / 3}, {x2, 1.0 / 3}});

//   auto actual_sol = model.GetPrimalDualPathFollowingSolution();

//   EXPECT_LE(5.0f - model.GetPrimalDualPathFollowingOptimum(), 1e-3f);
//   EXPECT_GE(5.0f - model.GetPrimalDualPathFollowingOptimum(), -1e-3f);

//   for (auto entry : expected_sol) {
//     EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
//     EXPECT_LE(entry.second - actual_sol[entry.first], 1e-3f);
//     EXPECT_GE(entry.second - actual_sol[entry.first], -1e-3f);
//   }
// }

TEST(LPModel, PrimalDualPathFollowingSolve3) {
  Parser parser;
  std::ifstream file("tests/test5.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();

  auto result = model.PrimalDualPathFollowingSolve(1e-9, {});

  EXPECT_EQ(result, SOLVED);
  std::cout << model.GetPrimalDualPathFollowingOptimum().ToString() << "\n";

  auto expected_sol = std::map<Variable, Num>({{x1, 3.75f}, {x2, 1.25f}});

  auto actual_sol = model.GetPrimalDualPathFollowingSolution();

  EXPECT_LE(23.75f - model.GetPrimalDualPathFollowingOptimum(), 1e-3f);
  EXPECT_GE(23.75f - model.GetPrimalDualPathFollowingOptimum(), -1e-3f);

  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(entry.second - actual_sol[entry.first], 1e-3f);
    EXPECT_GE(entry.second - actual_sol[entry.first], -1e-3f);
  }
}
