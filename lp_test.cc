#include "lp.h"
#include "parser.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT);
  c1.SetCompare(7.0f);
  c1.expression = x1 + x2;
  c1.SetEquationType(Constraint::Type::EQ);
  model.AddConstraint(c1);
  c2.SetCompare(4.0f);
  c2.expression = x1 - 2.0f * x2;
  c2.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c2);
  c3.SetCompare(0.0f);
  c3.expression = x1;
  c3.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c3);
  OptimizationObject opt(FLOAT);
  opt.expression = -2.0f * x1 + 3.0f * x2;
  opt.SetOptType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);

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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT);
  c1.SetCompare(12.0f);
  c1.expression = 2.0f * x1 + x2;
  c1.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c1);
  c2.SetCompare(9.0f);
  c2.expression = x1 + 2.0f * x2;
  c2.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c2);
  c3.SetCompare(0.0f);
  c3.expression = x1;
  c3.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x2;
  c4.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c4);
  OptimizationObject opt(FLOAT);
  opt.expression = -1.0f * x1 - x2;
  opt.SetOptType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);

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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT);
  c1.SetCompare(12.0f);
  c1.expression = 2.0f * x1 + 1.0f * x2;
  c1.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c1);
  c2.SetCompare(9.0f);
  c2.expression = 1.0f * x1 + 2.0f * x2;
  c2.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c2);
  c3.SetCompare(0.0f);
  c3.expression = x1;
  c3.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x2;
  c4.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c4);
  OptimizationObject opt(FLOAT);
  opt.expression = -1.0f * x1 - 1.0f * x2;
  opt.SetOptType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);
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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT);
  c1.SetCompare(12.0f);
  c1.expression = 2.0f * x1 + 1.0f * x2;
  c1.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c1);
  c2.SetCompare(9.0f);
  c2.expression = (1.0f * x1) + (2.0f * x2);
  c2.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c2);
  c3.SetCompare(0.0f);
  c3.expression = x1;
  c3.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x2;
  c4.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c4);
  OptimizationObject opt(FLOAT);
  opt.expression = -1.0f * x1 - 1.0f * x2;
  opt.SetOptType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);
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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT), c5(FLOAT);
  c1.SetCompare(1.0f);
  c1.expression = x1 + x2;
  c1.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c1);
  c2.SetCompare(1.0f);
  c2.expression = 2.0f * x1 - x2;
  c2.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c2);
  c3.SetCompare(2.0f);
  c3.expression = 3.0f * x2;
  c3.SetEquationType(Constraint::Type::LE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x1;
  c4.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c4);
  c5.SetCompare(0.0f);
  c5.expression = x2;
  c5.SetEquationType(Constraint::Type::GE);
  model.AddConstraint(c5);
  OptimizationObject opt(FLOAT);
  opt.expression = (6.0f * x1) + (3.0f * x2);
  opt.SetOptType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);
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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT);
  c1.expression = x1 + x2;
  c1.equation_type = Constraint::Type::LE;
  c1.compare = 5.0f;
  c2.expression = 10.0f * x1 + 6.0f * x2;
  c2.equation_type = Constraint::Type::LE;
  c2.compare = 45.0f;
  model.AddConstraint(c1);
  model.AddConstraint(c2);
  c3.expression = x1;
  c3.equation_type = Constraint::Type::GE;
  model.AddConstraint(c3);
  c4.expression = x2;
  c4.equation_type = Constraint::Type::GE;
  model.AddConstraint(c4);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MAX);
  obj.expression = 5.0f * x1 + 4.0f * x2;
  EXPECT_EQ(obj.expression.constant, kFloatZero);
  model.SetOptimizationObject(obj);

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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT), c5(FLOAT);
  c1.expression = 1.0f * x1 + 2.0f * x2;
  c1.equation_type = Constraint::Type::LE;
  c1.compare = 8.0f;
  c2.expression = 4.0f * x1;
  c2.equation_type = Constraint::Type::LE;
  c2.compare = 16.0f;
  c3.expression = 4.0f * x2;
  c3.equation_type = Constraint::Type::LE;
  c3.compare = 12.0f;
  model.AddConstraint(c1);
  model.AddConstraint(c2);
  model.AddConstraint(c3);
  c4.expression = x1;
  c4.equation_type = Constraint::Type::GE;
  model.AddConstraint(c4);
  c5.expression = x2;
  c5.equation_type = Constraint::Type::GE;
  model.AddConstraint(c5);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MAX);
  obj.expression = 2.0f * x1 + 3.0f * x2;
  EXPECT_EQ(obj.expression.constant, kFloatZero);
  model.SetOptimizationObject(obj);

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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT), c5(FLOAT);
  c1.expression = 1.0f * x1 + 2.0f * x2;
  c1.equation_type = Constraint::Type::LE;
  c1.compare = 8.0f;
  c2.expression = 4.0f * x1;
  c2.equation_type = Constraint::Type::LE;
  c2.compare = 16.0f;
  c3.expression = 4.0f * x2;
  c3.equation_type = Constraint::Type::LE;
  c3.compare = 12.0f;
  model.AddConstraint(c1);
  model.AddConstraint(c2);
  model.AddConstraint(c3);
  c4.expression = x1;
  c4.equation_type = Constraint::Type::GE;
  model.AddConstraint(c4);
  c5.expression = x2;
  c5.equation_type = Constraint::Type::GE;
  model.AddConstraint(c5);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MAX);
  obj.expression = 2.0f * x1 + 3.0f * x2;
  EXPECT_EQ(obj.expression.constant, kFloatZero);
  model.SetOptimizationObject(obj);

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
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT), c5(FLOAT);
  c1.expression = 1.0f * x1 + 2.0f * x2;
  c1.equation_type = Constraint::Type::LE;
  c1.compare = 1.0f;
  c2.expression = 4.0f * x1;
  c2.equation_type = Constraint::Type::LE;
  c2.compare = 10.0f;
  c3.expression = 4.0f * x2;
  c3.equation_type = Constraint::Type::LE;
  c3.compare = 9.0f;
  model.AddConstraint(c1);
  model.AddConstraint(c2);
  model.AddConstraint(c3);
  c4.expression = x1;
  c4.equation_type = Constraint::Type::GE;
  model.AddConstraint(c4);
  c5.expression = x2;
  c5.equation_type = Constraint::Type::GE;
  model.AddConstraint(c5);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MAX);
  obj.expression = 2.0f * x1 + 3.0f * x2;
  EXPECT_EQ(obj.expression.constant, kFloatZero);
  model.SetOptimizationObject(obj);

  model.ToStandardForm();
  model.ToSlackForm();

  auto result = model.Solve();
  EXPECT_EQ(result, SOLVED);

  auto base_variables = model.GetBaseVariables();

  LPModel raw_model;
  raw_model.SetOptimizationObject(obj);
  c1.compare = 8.0f;
  c2.compare = 16.0f;
  c3.compare = 12.0f;
  raw_model.AddConstraint(c1);
  raw_model.AddConstraint(c2);
  raw_model.AddConstraint(c3);
  raw_model.AddConstraint(c4);
  raw_model.AddConstraint(c5);

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
  LPModel model;
  Variable x1("x1"), x2("x2"), x3("x3"), x4("x4");
  Variable b0("base0"), b1("base1"), b2("base2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT), c5(FLOAT), c6(FLOAT),
      c7(FLOAT);
  c1.expression = 1000.0f * x1 + 1500.0f * x2 + 1750.0f * x3 + 3250.0f * x4;
  c1.equation_type = Constraint::Type::GE;
  c1.compare = 4000.0f;
  c2.expression = .6f * x1 + .27f * x2 + .68f * x3 + .3f * x4;
  c2.equation_type = Constraint::Type::GE;
  c2.compare = 1.0f;
  c3.expression = 17.5f * x1 + 7.7f * x2 + 30.0f * x4;
  c3.equation_type = Constraint::Type::GE;
  c3.compare = 30.0f;
  model.AddConstraint(c1);
  model.AddConstraint(c2);
  model.AddConstraint(c3);
  c4.expression = x1;
  c4.equation_type = Constraint::Type::GE;
  model.AddConstraint(c4);
  c5.expression = x2;
  c5.equation_type = Constraint::Type::GE;
  model.AddConstraint(c5);
  c6.expression = x3;
  c6.equation_type = Constraint::Type::GE;
  model.AddConstraint(c6);
  c7.expression = x4;
  c7.equation_type = Constraint::Type::GE;
  model.AddConstraint(c7);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MIN);
  obj.expression = .8f * x1 + .5f * x2 + .9f * x3 + 1.5f * x4;
  EXPECT_EQ(obj.expression.constant, kFloatZero);
  model.SetOptimizationObject(obj);

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
  LPModel model;
  Variable x1("x1"), x2("x2"), x3("x3"), x4("x4");
  Variable b0("base0"), b1("base1"), b2("base2");
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT), c5(FLOAT), c6(FLOAT),
      c7(FLOAT);
  c1.expression = 1000.0f * x1 + 1500.0f * x2 + 1750.0f * x3 + 3250.0f * x4;
  c1.equation_type = Constraint::Type::GE;
  c1.compare = 4000.0f;
  c2.expression = .6f * x1 + .27f * x2 + .68f * x3 + .3f * x4;
  c2.equation_type = Constraint::Type::GE;
  c2.compare = 1.0f;
  c3.expression = 17.5f * x1 + 7.7f * x2 + 30.0f * x4;
  c3.equation_type = Constraint::Type::GE;
  c3.compare = 30.0f;
  model.AddConstraint(c1);
  model.AddConstraint(c2);
  model.AddConstraint(c3);
  c4.expression = x1;
  c4.equation_type = Constraint::Type::GE;
  model.AddConstraint(c4);
  c5.expression = x2;
  c5.equation_type = Constraint::Type::GE;
  model.AddConstraint(c5);
  c6.expression = x3;
  c6.equation_type = Constraint::Type::GE;
  model.AddConstraint(c6);
  c7.expression = x4;
  c7.equation_type = Constraint::Type::GE;
  model.AddConstraint(c7);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MIN);
  obj.expression = .8f * x1 + .5f * x2 + .9f * x3 + 1.5f * x4;
  EXPECT_EQ(obj.expression.constant, kFloatZero);
  model.SetOptimizationObject(obj);

  model.ToStandardForm();
  auto result = model.ColumnGenerationSolve();
  EXPECT_EQ(result, SOLVED);
}
