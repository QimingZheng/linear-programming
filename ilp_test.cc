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