#include "base.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(Variable, Variable) {
  Variable undefined;
  Variable x1("x1"), x2("x2"), x3("x1"), x4("x1", INTEGER);
  EXPECT_EQ(undefined.ToString(), kUndefined);
  EXPECT_EQ(x1 == x3, true);
  EXPECT_EQ(x1 == x2, false);
  EXPECT_EQ(x1 == x4, false);
}

TEST(Num, Num) {
  Num float_1(1.0f);
  Num int_1(1);
  EXPECT_EQ(float_1 == int_1, false);
  EXPECT_EQ(int_1 * float_1 == float_1, true);
  EXPECT_EQ(int_1 + float_1, Num(2.0f));

  Num float_2(2.0f), float_4(4.0f);
  Num int_2(2), int_4(4);
  EXPECT_EQ(float_2 * float_2 == float_4, true);
  EXPECT_EQ(int_2 * int_2 == int_4, true);
  EXPECT_EQ(int_2 * float_2 == float_4, true);
  EXPECT_EQ(float_4 / float_2 == float_2, true);
  EXPECT_EQ(float_4 / int_2 == float_2, true);
  EXPECT_EQ(int_4 / int_2 == int_2, true);
}

TEST(Expression, Expression) {
  Expression exp1(1.0f);
  Expression exp2(1);
  Variable integer("x1", INTEGER), floating("x2", FLOAT);
  Expression exp3(integer), exp4(floating);
  EXPECT_EQ(exp1.ToString(), "1.000000");
  EXPECT_EQ(exp2.ToString(), "1");
  EXPECT_EQ(integer.type, INTEGER);
  EXPECT_EQ(exp3.GetCoeffOf(integer) == 1, true);
  EXPECT_EQ(exp4.GetCoeffOf(floating) == 1.0f, true);
  EXPECT_EQ(exp3.ToString(), "1 * x1 + 0");
  EXPECT_EQ(exp4.ToString(), "1.000000 * x2 + 0.000000");
}

TEST(Expression, SetCoeffOf) {
  Expression exp(0.0f);
  EXPECT_EQ(exp.ToString(), "0.000000");
  exp.constant = 1.0f;
  EXPECT_EQ(exp.ToString(), "1.000000");
  auto x1 = Variable("x1");
  auto x2 = Variable("x2");
  exp.SetCoeffOf(x1, 0.5f);
  EXPECT_EQ(exp.ToString(), "0.500000 * x1 + 1.000000");
  exp.constant = 0.0f;
  EXPECT_EQ(exp.ToString(), "0.500000 * x1 + 0.000000");
  exp.constant = 1.0f;
  exp.SetCoeffOf(x1, 1.0f);
  EXPECT_EQ(exp.ToString(), "1.000000 * x1 + 1.000000");
  exp.SetCoeffOf(x2, 0.0f);
  EXPECT_EQ(exp.ToString(), "1.000000 * x1 + 1.000000");
}

TEST(Expression, Add) {
  Expression exp1(0.0f), exp2(0.0f);
  exp1.constant = 1.0f;
  auto x1 = Variable("x1");
  auto x2 = Variable("x2");
  exp1.SetCoeffOf(x1, 0.5f);
  exp2.constant = 2.0f;
  exp2.SetCoeffOf(x2, 2.0f);
  exp1 += exp2;
  EXPECT_EQ(exp1.ToString(), "0.500000 * x1 + 2.000000 * x2 + 3.000000");
}

TEST(Expression, Multiply) {
  Expression exp(0.0f);
  exp.constant = 1.0f;
  auto x1 = Variable("x1");
  auto x2 = Variable("x2");
  exp.SetCoeffOf(x1, 0.5f);
  exp.SetCoeffOf(x2, 2.0f);
  exp *= 2.0f;
  EXPECT_EQ(exp.ToString(), "1.000000 * x1 + 4.000000 * x2 + 2.000000");
}

TEST(Expression, ReplaceVariableWithExpression) {
  Expression exp(0.0f);
  EXPECT_EQ(exp.ToString(), "0.000000");
  exp.constant = 1.0f;
  EXPECT_EQ(exp.ToString(), "1.000000");
  auto x1 = Variable("x1");
  auto x2 = Variable("x2");
  exp.SetCoeffOf(x1, 0.5f);
  EXPECT_EQ(exp.ToString(), "0.500000 * x1 + 1.000000");
  Expression sub(0.0f);
  sub.constant = 2.0f;
  sub.SetCoeffOf(x2, 2.0f);
  ReplaceVariableWithExpression(exp, x1, sub);
  EXPECT_EQ(exp.ToString(), "1.000000 * x2 + 2.000000");
}

TEST(Expression, GetOrSetCoeffOf) {
  Expression exp(0.0f);
  EXPECT_EQ(exp.ToString(), "0.000000");
  auto x1 = Variable("x1");
  auto x2 = Variable("x2");
  exp.SetCoeffOf(x1, 0.5f);
  EXPECT_EQ(exp.GetCoeffOf(x1), 0.5f);
  EXPECT_EQ(exp.GetCoeffOf(x2), 0.0f);
  exp.SetCoeffOf(x2, 1.0f);
  EXPECT_EQ(exp.GetCoeffOf(x2), 1.0f);
}

TEST(Constraint, Constructor) {
  Constraint constraint(FLOAT);
  auto x1 = Variable("x1");
  constraint.expression = x1;
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 0.000000 = 0.000000");
  constraint.SetConstant(1.0f);
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 1.000000 = 0.000000");
  constraint.SetCompare(1.0f);
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 1.000000 = 1.000000");
  constraint.SetEquationType(Constraint::Type::GE);
  EXPECT_EQ(constraint.ToString(), "1.000000 * x1 + 1.000000 >= 1.000000");
}

TEST(OptimizationObject, Constructor) {
  OptimizationObject obj(FLOAT);
  auto x1 = Variable("x1");
  auto x2 = Variable("x2");
  obj.expression = x1;
  EXPECT_EQ(obj.ToString(), "min 1.000000 * x1 + 0.000000");
  obj.expression = x1 + x2;
  EXPECT_EQ(obj.ToString(), "min 1.000000 * x1 + 1.000000 * x2 + 0.000000");
  obj.SetOptType(OptimizationObject::Type::MAX);
  EXPECT_EQ(obj.ToString(), "max 1.000000 * x1 + 1.000000 * x2 + 0.000000");
}
