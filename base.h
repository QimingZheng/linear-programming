/*
 * Created on Sun Jan 07 2024
 *
 * Copyright (c) 2024 - Qiming Zheng
 *
 * This file defines the interface of basic algebra arithmetic operations.
 *
 */
#pragma once

#include <math.h>
#include <sys/time.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

enum DataType {
  UNKNOWN,
  FLOAT,
  INTEGER,
};

const std::string kUndefined = "undefined";

typedef double real_t;

struct Variable {
 public:
  Variable() : variable_name(kUndefined) {}
  Variable(std::string name, DataType data_type = FLOAT)
      : variable_name(name), type(data_type) {}

  std::string ToString() const { return variable_name; }

  bool IsUndefined() const { return variable_name == kUndefined; }

  void To(DataType type) {
    if (this->type == type) return;
    if (this->type == DataType::UNKNOWN || type == DataType::UNKNOWN)
      throw std::runtime_error("Cannot convert unknown variable type");
    this->type = type;
  }

  std::string variable_name;
  DataType type;
};

bool operator==(const Variable &lhs, const Variable &rhs);

bool operator!=(const Variable &lhs, const Variable &rhs);

bool operator<(const Variable &lhs, const Variable &rhs);

const float kEpsilonF = 1e-6f;

struct Num {
 public:
  Num() : type(DataType::UNKNOWN) {}
  Num(real_t val) : type(FLOAT), float_value(val) {}
  Num(int val) : type(INTEGER), int_value(val) {}

  bool IsZero() const {
    switch (type) {
      case FLOAT:
        return abs(float_value) < kEpsilonF;
      case INTEGER:
        return int_value == 0;
      default:
        throw std::runtime_error("IsZero error\n");
    }
  }

  bool IsOne() const {
    switch (type) {
      case FLOAT:
        return abs(float_value - 1.0) < kEpsilonF;
      case INTEGER:
        return int_value == 1;
      default:
        throw std::runtime_error("IsZero error\n");
    }
  }

  bool IsNegative() const {
    switch (type) {
      case FLOAT:
        return float_value < -kEpsilonF;
      case INTEGER:
        return int_value < 0;
      default:
        throw std::runtime_error("IsZero error\n");
    }
  }

  bool IsNonNegative() const {
    switch (type) {
      case FLOAT:
        return float_value >= -kEpsilonF;
      case INTEGER:
        return int_value >= 0;
      default:
        throw std::runtime_error("IsZero error\n");
    }
  }

  std::string ToString() const {
    switch (type) {
      case FLOAT:
        return std::to_string(float_value);
      case INTEGER:
        return std::to_string(int_value);
      default:
        throw std::runtime_error("ToString error\n");
    }
    throw std::runtime_error("Unknown num data type");
  }

  void To(DataType type) {
    if (this->type == type) return;
    this->type = type;
    if (this->type == DataType::UNKNOWN || type == DataType::UNKNOWN)
      throw std::runtime_error("Cannot convert unknown data type");
    switch (type) {
      case FLOAT:
        float_value = int_value;
        break;

      case INTEGER:
        int_value = float_value;
        break;

      default:
        throw std::runtime_error("Cannot convert unknown num type");
    }
  }

  DataType type;
  real_t float_value;
  int int_value;
};

const Num kIntZero = Num(0);
const Num kFloatZero = Num(0.0f);
const Num kIntOne = Num(1);
const Num kFloatOne = Num(1.0f);
const Num kFloatMax = Num(std::numeric_limits<real_t>::max());
const Num kFloatMin = Num(std::numeric_limits<real_t>::min());
const Num kIntMax = Num(std::numeric_limits<int>::max());
const Num kIntMin = Num(std::numeric_limits<int>::min());
const Num kEpsilon = Num(kEpsilonF);

bool operator==(const Num &lhs, const Num &rhs);

bool operator!=(const Num &lhs, const Num &rhs);

void BinaryOp(Num &lhs, const Num rhs, char op);

void operator*=(Num &lhs, const Num rhs);
Num operator*(const Num lhs, const Num rhs);

void operator+=(Num &lhs, const Num rhs);
Num operator+(const Num lhs, const Num rhs);

void operator/=(Num &lhs, const Num rhs);
Num operator/(const Num lhs, const Num rhs);

void operator-=(Num &lhs, const Num rhs);
Num operator-(const Num lhs, const Num rhs);

bool operator<(const Num lhs, const Num rhs);
bool operator<=(const Num lhs, const Num rhs);

bool operator>(const Num lhs, const Num rhs);
bool operator>=(const Num lhs, const Num rhs);
Num operator-(const Num num);

struct Expression {
 public:
  Expression(Num num) : constant(num) {}
  Expression(Variable var)
      : variable_coeff({{var, (var.type == FLOAT ? kFloatOne : kIntOne)}}),
        constant(var.type == FLOAT ? kFloatZero : kIntZero) {}

  Num GetCoeffOf(Variable var) {
    if (variable_coeff.find(var) != variable_coeff.end())
      return variable_coeff.find(var)->second;
    return Num(var.type == FLOAT ? kFloatZero : kIntZero);
  }
  void SetCoeffOf(Variable var, Num coeff) {
    if (variable_coeff.find(var) != variable_coeff.end())
      variable_coeff.erase(var);
    if (coeff == kIntZero or coeff == kFloatZero) {
      return;
    }
    variable_coeff.insert(std::make_pair(var, coeff));
  }

  std::string ToString() const {
    std::string ret = "";
    for (auto entry : variable_coeff) {
      ret += entry.second.ToString() + " * " + entry.first.ToString() + " + ";
    }
    return ret + constant.ToString();
  }

  std::map<Variable, Num> variable_coeff;
  Num constant;
};

bool operator==(const Expression lhs, const Expression rhs);

bool operator!=(const Expression lhs, const Expression rhs);

Expression operator*(const Variable var, const Num num);
Expression operator*(const Num num, const Variable var);

void operator+=(Expression &lhs, const Expression rhs);
Expression operator+(const Expression exp1, const Expression exp2);

void operator-=(Expression &lhs, const Expression rhs);
Expression operator-(const Expression exp1, const Expression exp2);

void operator*=(Expression &exp, const Num multiplier);
Expression operator*(const Expression exp1, const Num multiplier);
Expression operator*(const Num multiplier, const Expression exp1);

void operator/=(Expression &exp, const Num multiplier);
Expression operator/(const Expression exp1, const Num multiplier);

void ReplaceVariableWithExpression(Expression &expression, Variable var,
                                   Expression substitution);

// \sum_{i} c_i * x_i + constant <=/>=/= compare.
struct Constraint {
 public:
  enum Type {
    LE,  // less than or equal to
    GE,  // greater than or equal to
    EQ,  // equal to
  };
  Constraint(DataType type) : data_type(type), expression(kFloatZero) {
    switch (type) {
      case FLOAT:
        expression = Expression(kFloatZero);
        compare = Num(kFloatZero);
        break;
      case INTEGER:
        expression = Expression(kIntZero);
        compare = Num(kIntZero);
        break;
      default:
        throw std::runtime_error("Unknown constraint data type");
    }
  }

  void SetConstant(Num constant) { expression.constant = constant; }
  void SetCompare(Num compare) { this->compare = compare; }
  void SetDataType(DataType type) { this->data_type = type; }
  void SetEquationType(Type type) { this->equation_type = type; }

  std::string ToString() {
    std::string ret = expression.ToString() + " ";
    switch (equation_type) {
      case LE:
        ret += "<=";
        break;
      case GE:
        ret += ">=";
        break;
      case EQ:
        ret += "=";
        break;

      default:
        break;
    }
    ret += " " + compare.ToString();
    return ret;
  }

  Expression expression;
  Num compare;
  Type equation_type = Type::EQ;
  DataType data_type;
};

bool operator==(const Constraint lhs, const Constraint rhs);

struct OptimizationObject {
 public:
  enum Type {
    MIN,
    MAX,
  };

  OptimizationObject(DataType type) : data_type(type), expression(kFloatZero) {
    switch (type) {
      case FLOAT:
        expression = Expression(kFloatZero);
        break;
      case INTEGER:
        expression = Expression(kIntZero);
        break;
      default:
        throw std::runtime_error("Unknown opt object data type");
    }
  }

  void SetOptType(Type type) { this->opt_type = type; }
  void SetDataType(DataType type) { this->data_type = type; }

  std::string ToString() {
    std::string ret = "";
    switch (opt_type) {
      case MIN:
        ret = "min ";
        break;
      case MAX:
        ret = "max ";
        break;

      default:
        break;
    }
    return ret + expression.ToString();
  }

  Expression expression;
  Type opt_type = MIN;
  DataType data_type;
};

class Timer {
 public:
  Timer() { gettimeofday(&start, NULL); }
  void Reset() { gettimeofday(&start, NULL); }
  void Stop() { gettimeofday(&end, NULL); }
  long Delta() {
    return (end.tv_usec - start.tv_usec) +
           (end.tv_sec - start.tv_sec) * 1000L * 1000L;
  }

 private:
  struct timeval start, end;
};