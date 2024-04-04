#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "lp.h"
#include "parser.h"

TEST(LPModel, TableauRevisiedSimplexSolve) {
  Parser parser;
  std::ifstream file("tests/test3.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();
  model.ToTableau(COLUMN_ONLY);

  EXPECT_EQ(model.TableauRevisiedSimplexSolve(), Result::SOLVED);

  EXPECT_EQ(model.GetTableauRevisedSimplexOptimum(), -7.0f);

  auto expected_sol = std::map<Variable, Num>({{x1, 5.0f}, {x2, 2.0f}});
  auto actual_sol = model.GetTableauRevisedSimplexSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(actual_sol[entry.first] - entry.second, kEpsilon);
    EXPECT_GE(actual_sol[entry.first] - entry.second, -kEpsilon);
  }
}

TEST(LPModel, TableauRevisiedSimplexSolve2) {
  Parser parser;
  std::ifstream file("tests/test8.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();
  model.ToTableau(COLUMN_ONLY);

  EXPECT_EQ(model.TableauRevisiedSimplexSolve(), Result::SOLVED);

  auto expected_sol = std::map<Variable, Num>({{x1, 4.0f}, {x2, 2.0f}});

  auto actual_sol = model.GetTableauRevisedSimplexSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(actual_sol[entry.first] - entry.second, kEpsilon);
    EXPECT_GE(actual_sol[entry.first] - entry.second, -kEpsilon);
  }
}

TEST(LPModel, TableauRevisiedSimplexSolve3) {
  Parser parser;
  std::ifstream file("tests/test5.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();
  model.ToTableau(COLUMN_ONLY);

  EXPECT_EQ(model.TableauRevisiedSimplexSolve(), Result::SOLVED);

  EXPECT_EQ(model.GetTableauRevisedSimplexOptimum(), 23.75f);

  auto expected_sol = std::map<Variable, Num>({{x1, 3.75f}, {x2, 1.25f}});

  auto actual_sol = model.GetTableauRevisedSimplexSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(actual_sol[entry.first] - entry.second, kEpsilon);
    EXPECT_GE(actual_sol[entry.first] - entry.second, -kEpsilon);
  }
}
