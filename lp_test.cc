#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "lp.h"
#include "parser.h"

TEST(LPModel, ToStandardForm) {
  Parser parser;
  std::ifstream file("tests/test0.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();

  EXPECT_EQ(
      model.ToString(),
      "max -3.000000 * subst0 + 3.000000 * subst1 + 2.000000 * x1 + -0.000000\n"
      "-1.000000 * subst0 + 1.000000 * subst1 + -1.000000 * x1 + "
      "-0.000000 <= -7.000000\n"
      "-2.000000 * subst0 + 2.000000 * subst1 + 1.000000 * x1 + 0.000000 "
      "<= 4.000000\n"
      "1.000000 * subst0 + -1.000000 * subst1 + 1.000000 * x1 + 0.000000 "
      "<= 7.000000\n");

  EXPECT_EQ(StandardFormSanityCheck(model), true);
  Expression exp(FLOAT);
  exp = Variable("subst0") - Variable("subst1");
  auto raw_variable_maps = std::map<Variable, Expression>({
      {x2, exp},
  });
  EXPECT_EQ(model.GetRawVariableExpression(), raw_variable_maps);
}

TEST(LPModel, ToSlackForm) {
  Parser parser;
  std::ifstream file("tests/test0.txt");
  LPModel model = parser.Parse(file);

  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(
      model.ToString(),
      "max -3.000000 * subst0 + 3.000000 * subst1 + 2.000000 * x1 + -0.000000\n"
      "-1.000000 * base0 + 1.000000 * subst0 + -1.000000 * subst1 + 1.000000 * "
      "x1 + -7.000000 = 0.000000\n"
      "-1.000000 * base1 + 2.000000 * subst0 + -2.000000 * subst1 + -1.000000 "
      "* x1 + 4.000000 = 0.000000\n"
      "-1.000000 * base2 + -1.000000 * subst0 + 1.000000 * subst1 + -1.000000 "
      "* x1 + 7.000000 = 0.000000\n");

  EXPECT_EQ(SlackFormSanityCheck(model), true);
}

TEST(LPModel, AddConstraint) {
  Parser parser;
  std::ifstream file("tests/test3.txt");
  LPModel model = parser.Parse(file);

  EXPECT_EQ(model.ToString(),
            "min -1.000000 * x1 + -1.000000 * x2 + 0.000000\n"
            "2.000000 * x1 + 1.000000 * x2 + 0.000000 <= 12.000000\n"
            "1.000000 * x1 + 2.000000 * x2 + 0.000000 <= 9.000000\n"
            "1.000000 * x1 + 0.000000 >= 0.000000\n"
            "1.000000 * x2 + 0.000000 >= 0.000000\n");

  model.ToStandardForm();

  EXPECT_EQ(model.ToString(),
            "max 1.000000 * x1 + 1.000000 * x2 + -0.000000\n"
            "2.000000 * x1 + 1.000000 * x2 + 0.000000 <= 12.000000\n"
            "1.000000 * x1 + 2.000000 * x2 + 0.000000 <= 9.000000\n");

  model.ToSlackForm();

  EXPECT_EQ(model.ToString(),
            "max 1.000000 * x1 + 1.000000 * x2 + -0.000000\n"
            "-1.000000 * base0 + -2.000000 * x1 + -1.000000 * x2 + 12.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + -1.000000 * x1 + -2.000000 * x2 + 9.000000 = "
            "0.000000\n");
}

TEST(LPModel, Pivot) {
  Parser parser;
  std::ifstream file("tests/test3.txt");
  LPModel model = parser.Parse(file);
  model.ToStandardForm();
  model.ToSlackForm();
  EXPECT_EQ(model.ToString(),
            "max 1.000000 * x1 + 1.000000 * x2 + -0.000000\n"
            "-1.000000 * base0 + -2.000000 * x1 + -1.000000 * x2 + 12.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + -1.000000 * x1 + -2.000000 * x2 + 9.000000 = "
            "0.000000\n");

  model.Pivot(Variable("base0"), Variable("x1"));
  EXPECT_EQ(model.ToString(),
            "max -0.500000 * base0 + 0.500000 * x2 + 6.000000\n"
            "-0.500000 * base0 + -1.000000 * x1 + -0.500000 * x2 + 6.000000 = "
            "0.000000\n"
            "0.500000 * base0 + -1.000000 * base1 + -1.500000 * x2 + 3.000000 "
            "= 0.000000\n");

  model.Pivot(Variable("base1"), Variable("x2"));
  EXPECT_EQ(model.ToString(),
            "max -0.333333 * base0 + -0.333333 * base1 + 7.000000\n"
            "-0.666667 * base0 + 0.333333 * base1 + -1.000000 * x1 + 5.000000 "
            "= 0.000000\n"
            "0.333333 * base0 + -0.666667 * base1 + -1.000000 * x2 + 2.000000 "
            "= 0.000000\n");
}

TEST(LPModel, Solve) {
  Parser parser;
  std::ifstream file("tests/test3.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");
  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(model.Solve(), Result::SOLVED);
  EXPECT_EQ(model.ToString(),
            "max -0.333333 * base0 + -0.333333 * base1 + 7.000000\n"
            "-0.666667 * base0 + 0.333333 * base1 + -1.000000 * x1 + 5.000000 "
            "= 0.000000\n"
            "0.333333 * base0 + -0.666667 * base1 + -1.000000 * x2 + 2.000000 "
            "= 0.000000\n");
  EXPECT_EQ(model.GetOptimum(), -7.0f);
  auto sol = std::map<Variable, Num>({{x1, 5.0f}, {x2, 2.0f}});
  EXPECT_EQ(model.GetSolution(), sol);
}

TEST(LPModel, Initialization) {
  // Test the two phase algorithm.
  /*
   * min 6 * x1 + 3 * x2
   * s.t.
   *    x1 + x2 >= 1
   *    2 * x1 - x2 >= 1
   *    3 * x2 <= 2
   */
  Parser parser;
  std::ifstream file("tests/test4.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");

  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(model.ToString(),
            "max -6.000000 * x1 + -3.000000 * x2 + -0.000000\n"
            "-1.000000 * base0 + 1.000000 * x1 + 1.000000 * x2 + -1.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + 2.000000 * x1 + -1.000000 * x2 + -1.000000 = "
            "0.000000\n"
            "-1.000000 * base2 + -3.000000 * x2 + 2.000000 = 0.000000\n");
  EXPECT_EQ(model.Initialize(), Result::SOLVED);
  EXPECT_EQ(model.ToString(),
            "max -6.000000 * base0 + 3.000000 * x2 + -6.000000\n"
            "1.000000 * base0 + -1.000000 * x1 + -1.000000 * x2 + 1.000000 = "
            "0.000000\n"
            "2.000000 * base0 + -1.000000 * base1 + -3.000000 * x2 + 1.000000 "
            "= 0.000000\n"
            "-1.000000 * base2 + -3.000000 * x2 + 2.000000 = 0.000000\n");

  EXPECT_EQ(model.Solve(), Result::SOLVED);
  EXPECT_EQ(model.GetOptimum(), 5.0f);
  auto expected_sol = std::map<Variable, Num>({{x1, 2.0f / 3}, {x2, 1.0f / 3}});
  auto actual_sol = model.GetSolution();
  EXPECT_EQ(expected_sol.size(), actual_sol.size());
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(entry.second - actual_sol[entry.first], 1e-6f);
    EXPECT_GE(entry.second - actual_sol[entry.first], -1e-6f);
  }
}

TEST(LPModel, GetSolution) {
  Parser parser;
  std::ifstream file("tests/test5.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");

  model.ToStandardForm();
  model.ToSlackForm();

  EXPECT_EQ(model.ToString(),
            "max 5.000000 * x1 + 4.000000 * x2 + 0.000000\n"
            "-1.000000 * base0 + -1.000000 * x1 + -1.000000 * x2 + 5.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + -10.000000 * x1 + -6.000000 * x2 + 45.000000 "
            "= 0.000000\n");

  model.Solve();

  auto expected_sol = std::map<Variable, Num>({{x1, 3.75f}, {x2, 1.25f}});

  EXPECT_EQ(model.GetOptimum(), 23.75f);
  auto actual_sol = model.GetSolution();
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(entry.second - actual_sol[entry.first], 1e-6f);
    EXPECT_GE(entry.second - actual_sol[entry.first], -1e-6f);
  }
}

TEST(LPModel, ToDualForm) {
  Parser parser;
  std::ifstream file("tests/test6.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");

  model.ToStandardForm();
  auto dual = model.ToDualForm();

  EXPECT_EQ(dual.ToString(),
            "min 8.000000 * dual0 + 16.000000 * dual1 + 12.000000 * dual2 + "
            "0.000000\n"
            "1.000000 * dual0 + 4.000000 * dual1 + 0.000000 >= 2.000000\n"
            "2.000000 * dual0 + 4.000000 * dual2 + 0.000000 >= 3.000000\n"
            "1.000000 * dual0 + 0.000000 >= 0.000000\n"
            "1.000000 * dual1 + 0.000000 >= 0.000000\n"
            "1.000000 * dual2 + 0.000000 >= 0.000000\n");
}

TEST(LPModel, GaussianElimination) {
  Parser parser;
  std::ifstream file("tests/test6.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");

  model.ToStandardForm();
  model.ToSlackForm();

  std::set<Variable> vars = {x1};
  model.GaussianElimination(vars);
  EXPECT_EQ(model.ToString(),
            "max 2.000000 * x1 + 3.000000 * x2 + 0.000000\n"
            "1.000000 * base0 + 1.000000 * x1 + 2.000000 * x2 + -8.000000 = "
            "0.000000\n"
            "4.000000 * base0 + -1.000000 * base1 + 8.000000 * x2 + -16.000000 "
            "= 0.000000\n"
            "-1.000000 * base2 + -4.000000 * x2 + 12.000000 = 0.000000\n");

  vars = {x2};
  model.GaussianElimination(vars);
  EXPECT_EQ(model.ToString(),
            "max 2.000000 * x1 + 3.000000 * x2 + 0.000000\n"
            "0.500000 * base0 + 0.500000 * x1 + 1.000000 * x2 + -4.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + -4.000000 * x1 + 16.000000 = 0.000000\n"
            "2.000000 * base0 + -1.000000 * base2 + 2.000000 * x1 + -4.000000 "
            "= 0.000000\n");
}

TEST(LPModel, DualSolve) {
  Parser parser;
  std::ifstream file("tests/test7.txt");
  LPModel model = parser.Parse(file);
  Variable x1("x1"), x2("x2");

  model.ToStandardForm();
  model.ToSlackForm();

  auto result = model.Solve();
  EXPECT_EQ(result, SOLVED);

  auto base_variables = model.GetBaseVariables();

  std::ifstream file2("tests/test8.txt");
  LPModel raw_model = parser.Parse(file2);

  raw_model.ToStandardForm();
  raw_model.ToSlackForm();
  result = raw_model.DualSolve(base_variables);
  EXPECT_EQ(result, SOLVED);

  auto expected_sol = std::map<Variable, Num>({{x1, 4.0f}, {x2, 2.0f}});
  auto actual_sol = raw_model.GetSolution();
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(entry.second - actual_sol[entry.first], 1e-6f);
    EXPECT_GE(entry.second - actual_sol[entry.first], -1e-6f);
  }
}

TEST(LPModel, DualSolve2) {
  Parser parser;
  std::ifstream file("tests/test9.txt");
  LPModel model = parser.Parse(file);

  Variable x1("x1"), x2("x2"), x3("x3"), x4("x4");
  Variable b0("base0"), b1("base1"), b2("base2");

  model.ToStandardForm();
  model.ToSlackForm();

  auto result = model.DualSolve({b0, b1, b2});
  EXPECT_EQ(result, SOLVED);
  EXPECT_LE(model.GetDualSolveOptimum() - Num(2.685733f), 1e-6f);
  EXPECT_GE(model.GetDualSolveOptimum() - Num(2.685733f), -1e-6f);

  auto expected_sol = std::map<Variable, Num>(
      {{x1, 0.704872f}, {x2, 2.074765f}, {x3, 0.0f}, {x4, 0.056302f}});
  auto actual_sol = model.GetSolution();

  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(entry.second - actual_sol[entry.first], 1e-6f);
    EXPECT_GE(entry.second - actual_sol[entry.first], -1e-6f);
  }
}

TEST(LPModel, ColumnGenerationSolve) {
  Parser parser;
  std::ifstream file("tests/test9.txt");
  LPModel model = parser.Parse(file);

  model.ToStandardForm();
  auto result = model.ColumnGenerationSolve();
  EXPECT_EQ(result, SOLVED);
}
