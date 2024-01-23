#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "parser.h"

TEST(Lexer, Scan) {
  Lexer lex;
  std::string input = "max 3 * x + 4 * y";
  std::vector<Token> tokens = {
      {Token::MAX, "max", 0, 3}, {Token::NUM, "3", 4, 5},
      {Token::MUL, "*", 6, 7},   {Token::VAR, "x", 8, 9},
      {Token::ADD, "+", 10, 11}, {Token::NUM, "4", 12, 13},
      {Token::MUL, "*", 14, 15}, {Token::VAR, "y", 16, 17},
  };
  EXPECT_EQ(lex.Scan(input), tokens);
  input = "st";
  tokens = {{Token::ST, "st", 0, 2}};
  EXPECT_EQ(lex.Scan(input), tokens);
  input = "3 * x + 4 * y -5<=60";
  tokens = {
      {Token::NUM, "3", 0, 1},    {Token::MUL, "*", 2, 3},
      {Token::VAR, "x", 4, 5},    {Token::ADD, "+", 6, 7},
      {Token::NUM, "4", 8, 9},    {Token::MUL, "*", 10, 11},
      {Token::VAR, "y", 12, 13},  {Token::SUB, "-", 14, 15},
      {Token::NUM, "5", 15, 16},  {Token::LE, "<=", 16, 18},
      {Token::NUM, "60", 18, 20},
  };
  EXPECT_EQ(lex.Scan(input), tokens);
  input = "-2.0 * x1 + 3.0 * x2";
  tokens = {
      {Token::SUB, "-", 0, 1},   {Token::NUM, "2.0", 1, 4},
      {Token::MUL, "*", 5, 6},   {Token::VAR, "x1", 7, 9},
      {Token::ADD, "+", 10, 11}, {Token::NUM, "3.0", 12, 15},
      {Token::MUL, "*", 16, 17}, {Token::VAR, "x2", 18, 20},
  };
  EXPECT_EQ(lex.Scan(input), tokens);
}

TEST(Parser, ParseOptimizationObject) {
  Parser parser;
  std::vector<Token> tokens = {
      {Token::MAX, "max", 0, 3}, {Token::NUM, "3", 4, 5},
      {Token::MUL, "*", 6, 7},   {Token::VAR, "x", 8, 9},
      {Token::ADD, "+", 10, 11}, {Token::NUM, "4", 12, 13},
      {Token::MUL, "*", 14, 15}, {Token::VAR, "y", 16, 17},
  };
  auto obj = parser.ParseOptimizationObject(tokens);
  EXPECT_EQ(obj.ToString(), "max 3.000000 * x + 4.000000 * y + 0.000000");
}

TEST(Parser, ParseConstraint) {
  Parser parser;
  std::vector<Token> tokens = {
      {Token::NUM, "3", 0, 1},    {Token::MUL, "*", 2, 3},
      {Token::VAR, "x", 4, 5},    {Token::ADD, "+", 6, 7},
      {Token::NUM, "4", 8, 9},    {Token::MUL, "*", 10, 11},
      {Token::VAR, "y", 12, 13},  {Token::SUB, "-", 14, 15},
      {Token::NUM, "5", 15, 16},  {Token::LE, "<=", 16, 18},
      {Token::NUM, "60", 18, 20},
  };
  auto con = parser.ParseConstraint(tokens);
  EXPECT_EQ(con.ToString(),
            "3.000000 * x + 4.000000 * y + -5.000000 <= 60.000000");
}

TEST(Parser, ParseExpression) {
  Parser parser;
  std::vector<Token> tokens = {
      {Token::NUM, "3", 0, 1},   {Token::MUL, "*", 2, 3},
      {Token::VAR, "x", 4, 5},   {Token::ADD, "+", 6, 7},
      {Token::NUM, "4", 8, 9},   {Token::MUL, "*", 10, 11},
      {Token::VAR, "y", 12, 13}, {Token::SUB, "-", 14, 15},
      {Token::NUM, "5", 15, 16},
  };
  auto con = parser.ParseExpression(tokens);
  EXPECT_EQ(con.ToString(), "3.000000 * x + 4.000000 * y + -5.000000");
  tokens = {
      {Token::SUB, "-", 0, 1},   {Token::NUM, "2.0", 1, 4},
      {Token::MUL, "*", 5, 6},   {Token::VAR, "x1", 7, 9},
      {Token::ADD, "+", 10, 11}, {Token::NUM, "3.0", 12, 15},
      {Token::MUL, "*", 16, 17}, {Token::VAR, "x2", 18, 20},
  };
  con = parser.ParseExpression(tokens);
  EXPECT_EQ(con.ToString(), "-2.000000 * x1 + 3.000000 * x2 + 0.000000");
}

TEST(Parser, Parse) {
  Parser parser;
  std::vector<Token> tokens = {
      // objective function:
      {Token::MAX, "max", 0, 3},
      {Token::NUM, "3", 4, 5},
      {Token::MUL, "*", 6, 7},
      {Token::VAR, "x", 8, 9},
      {Token::ADD, "+", 10, 11},
      {Token::NUM, "4", 12, 13},
      {Token::MUL, "*", 14, 15},
      {Token::VAR, "y", 16, 17},
      {Token::EOL, "\n", -1, -1},

      // st.
      {Token::ST, "st", -1, -1},
      {Token::EOL, "\n", -1, -1},

      // constraint:
      {Token::NUM, "3", 0, 1},
      {Token::MUL, "*", 2, 3},
      {Token::VAR, "x", 4, 5},
      {Token::ADD, "+", 6, 7},
      {Token::NUM, "4", 8, 9},
      {Token::MUL, "*", 10, 11},
      {Token::VAR, "y", 12, 13},
      {Token::SUB, "-", 14, 15},
      {Token::NUM, "5", 15, 16},
      {Token::LE, "<=", 16, 18},
      {Token::NUM, "60", 18, 20},
  };
  auto model = parser.Parse(tokens);
  EXPECT_EQ(model.ToString(),
            "max 3.000000 * x + 4.000000 * y + 0.000000\n"
            "3.000000 * x + 4.000000 * y + -5.000000 <= 60.000000\n");
}

TEST(Parser, Parse2) {
  Parser parser;
  std::ifstream file("tests/test1.txt", std::ifstream::in);
  auto model = parser.Parse(file);
  EXPECT_EQ(
      model.ToString(),
      "max 1.000000 * x1 + 14.000000 * x2 + 6.000000 * x3 + 0.000000\n"
      "1.000000 * x1 + 1.000000 * x2 + 1.000000 * x3 + 0.000000 <= 3.000000\n"
      "1.000000 * x1 + 0.000000 <= 2.000000\n"
      "1.000000 * x3 + 0.000000 <= 3.000000\n"
      "3.000000 * x2 + 1.000000 * x3 + 0.000000 <= 6.000000\n"
      "1.000000 * x1 + 0.000000 >= 0.000000\n"
      "1.000000 * x2 + 0.000000 >= 0.000000\n"
      "1.000000 * x3 + 0.000000 >= 0.000000\n");
}