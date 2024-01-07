#include "lp.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(Constraint, Constructor) {
  Constraint constraint;
  auto x1 = Variable("x1");
  constraint.expression = x1;
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 0.000000 = 0.000000");
  constraint.SetConstant(1.0f);
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 1.000000 = 0.000000");
  constraint.SetCompare(1.0f);
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 1.000000 = 1.000000");
  constraint.SetType(Constraint::Type::GE);
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 1.000000 >= 1.000000");
}

TEST(OptimizationObject, Constructor) {
  OptimizationObject obj;
  auto x1 = Variable("x1");
  auto x2 = Variable("x2");
  obj.expression = x1;
  EXPECT_EQ(obj.ToString(), "min 1.000000 * x1 + 0.000000");
  obj.expression = x1 + x2;
  EXPECT_EQ(obj.ToString(), "min 1.000000 * x1 + 1.000000 * x2 + 0.000000");
  obj.SetType(OptimizationObject::Type::MAX);
  EXPECT_EQ(obj.ToString(), "max 1.000000 * x1 + 1.000000 * x2 + 0.000000");
}

TEST(LPModel, AddConstraint) {
  LPModel model;
  Variable x1("x1"), x2("x2");
  Constraint c1, c2, c3, c4;
  c1.SetCompare(12.0f);
  c1.expression = 2.0f * x1 + x2;
  c1.SetType(Constraint::Type::LE);
  model.AddConstraint(c1);
  c2.SetCompare(9.0f);
  c2.expression = x1 + 2.0f * x2;
  c2.SetType(Constraint::Type::LE);
  model.AddConstraint(c2);
  c3.SetCompare(0.0f);
  c3.expression = x1;
  c3.SetType(Constraint::Type::GE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x2;
  c4.SetType(Constraint::Type::GE);
  model.AddConstraint(c4);
  OptimizationObject opt;
  opt.expression = -1.0f * x1 - x2;
  opt.SetType(OptimizationObject::Type::MIN);
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

  model.ToRelaxedForm();

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
  Constraint c1, c2, c3, c4;
  c1.SetCompare(12.0f);
  c1.expression = 2.0f * x1 + 1.0f * x2;
  c1.SetType(Constraint::Type::LE);
  model.AddConstraint(c1);
  c2.SetCompare(9.0f);
  c2.expression = 1.0f * x1 + 2.0f * x2;
  c2.SetType(Constraint::Type::LE);
  model.AddConstraint(c2);
  c3.SetCompare(0.0f);
  c3.expression = x1;
  c3.SetType(Constraint::Type::GE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x2;
  c4.SetType(Constraint::Type::GE);
  model.AddConstraint(c4);
  OptimizationObject opt;
  opt.expression = -1.0f * x1 - 1.0f * x2;
  opt.SetType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);
  model.ToStandardForm();
  model.ToRelaxedForm();
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
  Constraint c1, c2, c3, c4;
  c1.SetCompare(12.0f);
  c1.expression = 2.0f * x1 + 1.0f * x2;
  c1.SetType(Constraint::Type::LE);
  model.AddConstraint(c1);
  c2.SetCompare(9.0f);
  c2.expression = (1.0f * x1) + (2.0f * x2);
  c2.SetType(Constraint::Type::LE);
  model.AddConstraint(c2);
  c3.SetCompare(0.0f);
  c3.expression = x1;
  c3.SetType(Constraint::Type::GE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x2;
  c4.SetType(Constraint::Type::GE);
  model.AddConstraint(c4);
  OptimizationObject opt;
  opt.expression = -1.0f * x1 - 1.0f * x2;
  opt.SetType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);
  model.ToStandardForm();
  model.ToRelaxedForm();

  EXPECT_EQ(model.Solve(), LPModel::Result::SOLVED);
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
  Constraint c1, c2, c3, c4, c5;
  c1.SetCompare(1.0f);
  c1.expression = x1 + x2;
  c1.SetType(Constraint::Type::GE);
  model.AddConstraint(c1);
  c2.SetCompare(1.0f);
  c2.expression = 2.0f * x1 - x2;
  c2.SetType(Constraint::Type::GE);
  model.AddConstraint(c2);
  c3.SetCompare(2.0f);
  c3.expression = 3.0f * x2;
  c3.SetType(Constraint::Type::LE);
  model.AddConstraint(c3);
  c4.SetCompare(0.0f);
  c4.expression = x1;
  c4.SetType(Constraint::Type::GE);
  model.AddConstraint(c4);
  c5.SetCompare(0.0f);
  c5.expression = x2;
  c5.SetType(Constraint::Type::GE);
  model.AddConstraint(c5);
  OptimizationObject opt;
  opt.expression = (6.0f * x1) + (3.0f * x2);
  opt.SetType(OptimizationObject::Type::MIN);
  model.SetOptimizationObject(opt);
  model.ToStandardForm();
  model.ToRelaxedForm();

  EXPECT_EQ(model.ToString(),
            "max -6.000000 * x1 + -3.000000 * x2 + -0.000000\n"
            "-1.000000 * base0 + 1.000000 * x1 + 1.000000 * x2 + -1.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + 2.000000 * x1 + -1.000000 * x2 + -1.000000 = "
            "0.000000\n"
            "-1.000000 * base2 + -3.000000 * x2 + 2.000000 = 0.000000\n");
  EXPECT_EQ(model.Initialize(), LPModel::Result::SOLVED);
  EXPECT_EQ(model.ToString(),
            "max -6.000000 * base0 + 3.000000 * x2 + -6.000000\n"
            "1.000000 * base0 + -1.000000 * x1 + -1.000000 * x2 + 1.000000 = "
            "0.000000\n"
            "2.000000 * base0 + -1.000000 * base1 + -3.000000 * x2 + 1.000000 "
            "= 0.000000\n"
            "-1.000000 * base2 + -3.000000 * x2 + 2.000000 = 0.000000\n");

  EXPECT_EQ(model.Solve(), LPModel::Result::SOLVED);
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