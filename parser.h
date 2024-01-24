/*
 * Created on Sun Jan 24 2024
 *
 * Copyright (c) 2024 - Qiming Zheng
 *
 * This file defines the interface of LP problem description file parser.
 *
 */
#pragma once

#include <assert.h>

#include "base.h"
#include "lp.h"

struct Token {
  enum TokenType {
    UNKNOWN = 0,
    VAR,  // [a-z|A-Z|_][a-z|A-Z|_|0-9]*
    NUM,
    // reserved key words:
    MAX,
    MIN,
    ST,  // st
    // equation type:
    EQ,
    GE,
    LE,
    // Arithmic operations:
    ADD,
    SUB,
    MUL,
    // DIV is not supported.
    EOL,  // end of one line.
    // other symbols:
    COMMA,
  };
  TokenType type;
  std::string lexim;
  int start_index;
  int end_index;
};

bool operator==(const Token a, const Token b) {
  return a.type == b.type and a.lexim == b.lexim and
         a.start_index == b.start_index and a.end_index == b.end_index;
}

bool operator!=(const Token a, const Token b) { return !(a == b); }

class Lexer {
 public:
  std::vector<Token> Scan(std::string input);

 private:
  bool IsLetter(char);
  bool IsDigit(char);
};

class Parser {
 public:
  Model Parse(std::vector<Token> tokens);
  Model Parse(std::string input);
  Model Parse(std::ifstream &file);

  OptimizationObject ParseOptimizationObject(std::vector<Token> tokens);
  Constraint ParseConstraint(std::vector<Token> tokens);
  Expression ParseExpression(std::vector<Token> tokens);
};
