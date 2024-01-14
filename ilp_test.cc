#include "ilp.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

class MockLPModel : public LPModel {
 public:
  MockLPModel() {}

  MOCK_METHOD1(IsBaseVariable, bool(Variable));
  MOCK_METHOD1(AddBaseVariable, void(Variable));

 private:
};

TEST(ILPModel, FindGomoryCut) {
  MockLPModel mock;
  ILPModel ilp_model;
  Constraint con(FLOAT);
  con.equation_type = Constraint::Type::EQ;
  Variable base("base8"), non_base("x0");
  con.expression = Num(1.2f) - 1.3f * base - 1.4f * non_base;

  using ::testing::_;
  using ::testing::Return;

  EXPECT_CALL(mock, IsBaseVariable(non_base)).WillOnce(Return(false));
  EXPECT_CALL(mock, IsBaseVariable(base)).WillOnce(Return(true));
  EXPECT_CALL(mock, AddBaseVariable(Variable("base0"))).WillOnce(Return());

  EXPECT_EQ(ilp_model.FindGomoryCut(mock, con).ToString(),
            "-1.000000 * base0 + 0.400000 * x0 + -0.200000 = 0.000000");
}

TEST(ILPModel, ToRelaxedLPModel) {
  ILPModel ilp_model;
  Variable x1("x1", INTEGER), x2("x2", INTEGER), x3("x3", INTEGER);
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT), c5(FLOAT);
  c1.expression = 1.2f * x1 + 2.3f * x2;
  c1.equation_type = Constraint::Type::LE;
  c2.expression = x2 + 2.0f * x3;
  c2.equation_type = Constraint::Type::GE;
  ilp_model.AddConstraint(c1);
  ilp_model.AddConstraint(c2);
  c3.expression = x1;
  c3.equation_type = Constraint::Type::GE;
  ilp_model.AddConstraint(c3);
  c4.expression = x2;
  c4.equation_type = Constraint::Type::GE;
  ilp_model.AddConstraint(c4);
  c5.expression = x3;
  c5.equation_type = Constraint::Type::GE;
  ilp_model.AddConstraint(c5);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MIN);
  obj.expression = x1 + x2 + x3;
  ilp_model.SetOptimizationObject(obj);

  auto lp_model = ilp_model.ToRelaxedLPModel();
  EXPECT_EQ(lp_model.ToString(),
            "max -1.000000 * x1 + -1.000000 * x2 + -1.000000 * x3 + -0.000000\n"
            "-1.000000 * base0 + -1.200000 * x1 + -2.300000 * x2 + 0.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + 1.000000 * x2 + 2.000000 * x3 + 0.000000 = "
            "0.000000\n");
}

TEST(ILPModel, BranchAndBoundSolve) {
  ILPModel ilp_model;
  Variable x1("x1", INTEGER), x2("x2", INTEGER);
  Constraint c1(FLOAT), c2(FLOAT), c3(FLOAT), c4(FLOAT);
  c1.expression = x1 + x2;
  c1.equation_type = Constraint::Type::LE;
  c1.compare = 5.0f;
  c2.expression = 10 * x1 + 6 * x2;
  c2.equation_type = Constraint::Type::LE;
  c2.compare = 45.0f;
  ilp_model.AddConstraint(c1);
  ilp_model.AddConstraint(c2);
  c3.expression = x1;
  c3.equation_type = Constraint::Type::GE;
  ilp_model.AddConstraint(c3);
  c4.expression = x2;
  c4.equation_type = Constraint::Type::GE;
  ilp_model.AddConstraint(c4);
  OptimizationObject obj(FLOAT);
  obj.SetOptType(OptimizationObject::Type::MAX);
  obj.expression = 5 * x1 + 4 * x2;
  ilp_model.SetOptimizationObject(obj);

  auto result = ilp_model.BranchAndBoundSolve();
  EXPECT_EQ(result, SOLVED);
  EXPECT_EQ(ilp_model.GetOptimum(), Num(23.0f));
  auto expected_sol = std::map<Variable, Num>({{x1, 3}, {x2, 2}});
  auto actual_sol = ilp_model.GetSolution();
  for (auto entry : expected_sol) {
    EXPECT_EQ(actual_sol.find(entry.first) != actual_sol.end(), true);
    EXPECT_LE(entry.second - actual_sol[entry.first], 1e-6f);
    EXPECT_GE(entry.second - actual_sol[entry.first], -1e-6f);
  }
}