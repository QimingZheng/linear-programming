#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "lp.h"
#include "parser.h"

TEST(LPModel, RevisedSimplexSolve) {
  Parser parser;
  std::ifstream file("tests/test3.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(model.RevisedSimplexSolve(), Result::SOLVED);

  EXPECT_EQ(model.GetRevisedSimplexOptimum(), -7.0f);

  auto expected_sol = std::map<Variable, Num>({{x1, 5.0f}, {x2, 2.0f}});
  auto actual_sol = model.GetRevisedSimplexSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(actual_sol[entry.first] - entry.second, kEpsilon);
    EXPECT_GE(actual_sol[entry.first] - entry.second, -kEpsilon);
  }
}

TEST(LPModel, RevisedSimplexSolve2) {
  Parser parser;
  std::ifstream file("tests/test8.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(model.RevisedSimplexSolve(), Result::SOLVED);

  auto expected_sol = std::map<Variable, Num>({{x1, 4.0f}, {x2, 2.0f}});

  auto actual_sol = model.GetRevisedSimplexSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(actual_sol[entry.first] - entry.second, kEpsilon);
    EXPECT_GE(actual_sol[entry.first] - entry.second, -kEpsilon);
  }
}

TEST(LPModel, RevisedSimplexSolve3) {
  Parser parser;
  std::ifstream file("tests/test5.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(model.RevisedSimplexSolve(), Result::SOLVED);

  EXPECT_EQ(model.GetRevisedSimplexOptimum(), 23.75f);

  auto expected_sol = std::map<Variable, Num>({{x1, 3.75f}, {x2, 1.25f}});

  auto actual_sol = model.GetRevisedSimplexSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(actual_sol[entry.first] - entry.second, kEpsilon);
    EXPECT_GE(actual_sol[entry.first] - entry.second, -kEpsilon);
  }
}

TEST(LPModel, RevisedSimplexSolve4) {
  Parser parser;
  std::ifstream file("tests/test4.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(model.RevisedSimplexSolve(), Result::SOLVED);

  EXPECT_EQ(model.GetRevisedSimplexOptimum(), 5.0f);

  auto expected_sol = std::map<Variable, Num>({{x1, 2.0f / 3}, {x2, 1.0f / 3}});

  auto actual_sol = model.GetRevisedSimplexSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(actual_sol[entry.first] - entry.second, kEpsilon);
    EXPECT_GE(actual_sol[entry.first] - entry.second, -kEpsilon);
  }
}