#include "ilp.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "parser.h"

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
  Parser parser;
  std::ifstream file("tests/test11.txt");
  ILPModel ilp_model = parser.Parse(file);
  Variable x1("x1", INTEGER), x2("x2", INTEGER), x3("x3", INTEGER);

  auto lp_model = ilp_model.ToRelaxedLPModel();
  EXPECT_EQ(lp_model.ToString(),
            "max -1.000000 * x1 + -1.000000 * x2 + -1.000000 * x3 + -0.000000\n"
            "-1.000000 * base0 + -1.200000 * x1 + -2.300000 * x2 + 0.000000 = "
            "0.000000\n"
            "-1.000000 * base1 + 1.000000 * x2 + 2.000000 * x3 + 0.000000 = "
            "0.000000\n");
}

TEST(ILPModel, BranchAndBoundSolve) {
  Parser parser;
  std::ifstream file("tests/test10.txt");
  ILPModel ilp_model = parser.Parse(file);
  Variable x1("x1", INTEGER), x2("x2", INTEGER);

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