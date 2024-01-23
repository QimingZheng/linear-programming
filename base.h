/*
 * Created on Sun Jan 07 2023
 *
 * Copyright (c) 2024 - Qiming Zheng
 *
 * This file defines the interface of basic algebra arithmetic operations.
 *
 */
#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>

enum DataType {
  UNKNOWN,
  FLOAT,
  INTEGER,
};

const std::string kUndefined = "undefined";

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

bool operator==(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name == rhs.variable_name and lhs.type == rhs.type;
}

bool operator!=(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name != rhs.variable_name or lhs.type != rhs.type;
}

bool operator<(const Variable &lhs, const Variable &rhs) {
  return lhs.type <= rhs.type and lhs.variable_name < rhs.variable_name;
}

struct Num {
 public:
  Num() : type(DataType::UNKNOWN) {}
  Num(float val) : type(FLOAT), float_value(val) {}
  Num(int val) : type(INTEGER), int_value(val) {}

  bool IsZero() {
    switch (type) {
      case FLOAT:
        return float_value == 0.0;
      case INTEGER:
        return int_value == 0;
    }
  }

  std::string ToString() const {
    switch (type) {
      case FLOAT:
        return std::to_string(float_value);
      case INTEGER:
        return std::to_string(int_value);
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
  float float_value;
  int int_value;
};

const Num kIntZero = Num(0);
const Num kFloatZero = Num(0.0f);
const Num kIntOne = Num(1);
const Num kFloatOne = Num(1.0f);

bool operator==(const Num &lhs, const Num &rhs) {
  if (lhs.type != rhs.type) return false;
  auto type = lhs.type;
  switch (type) {
    case FLOAT:
      return lhs.float_value == rhs.float_value;

    case INTEGER:
      return lhs.int_value == rhs.int_value;
  }
}

bool operator!=(const Num &lhs, const Num &rhs) { return !(lhs == rhs); }

void BinaryOp(Num &lhs, const Num rhs, char op) {
  switch (lhs.type) {
    case FLOAT:
      switch (op) {
        case '*':
          lhs.float_value =
              lhs.float_value *
              (rhs.type == FLOAT ? rhs.float_value : rhs.int_value);
          return;

        case '+':
          lhs.float_value =
              lhs.float_value +
              (rhs.type == FLOAT ? rhs.float_value : rhs.int_value);
          return;

        case '/':
          lhs.float_value =
              lhs.float_value /
              (rhs.type == FLOAT ? rhs.float_value : rhs.int_value);
          return;

        case '-':
          lhs.float_value =
              lhs.float_value -
              (rhs.type == FLOAT ? rhs.float_value : rhs.int_value);
          return;

        default:
          throw std::runtime_error("Unknown binary operator " + op);
      }
      break;

    case INTEGER:
      switch (op) {
        case '*':
          lhs.int_value = lhs.int_value * rhs.int_value;
          return;

        case '+':
          lhs.int_value = lhs.int_value + rhs.int_value;
          return;

        case '/':
          lhs.int_value = lhs.int_value / rhs.int_value;
          return;

        case '-':
          lhs.int_value = lhs.int_value - rhs.int_value;
          return;

        default:
          throw std::runtime_error("Unknown binary operator " + op);
      }
      break;
  }
  throw std::runtime_error("Unknown binary element type");
}

void operator*=(Num &lhs, const Num rhs) {
  if (lhs.type == INTEGER and rhs.type == FLOAT) {
    lhs.type = FLOAT;
    lhs.float_value = lhs.int_value;
  }
  BinaryOp(lhs, rhs, '*');
}
Num operator*(const Num lhs, const Num rhs) {
  Num num = lhs;
  num *= rhs;
  return num;
}

void operator+=(Num &lhs, const Num rhs) {
  if (lhs.type == INTEGER and rhs.type == FLOAT) {
    lhs.type = FLOAT;
    lhs.float_value = lhs.int_value;
  }
  BinaryOp(lhs, rhs, '+');
}
Num operator+(const Num lhs, const Num rhs) {
  Num num = lhs;
  num += rhs;
  return num;
}

void operator/=(Num &lhs, const Num rhs) {
  if (lhs.type == INTEGER and rhs.type == FLOAT) {
    lhs.type = FLOAT;
    lhs.float_value = lhs.int_value;
  }
  BinaryOp(lhs, rhs, '/');
}
Num operator/(const Num lhs, const Num rhs) {
  Num num = lhs;
  num /= rhs;
  return num;
}

void operator-=(Num &lhs, const Num rhs) {
  if (lhs.type == INTEGER and rhs.type == FLOAT) {
    lhs.type = FLOAT;
    lhs.float_value = lhs.int_value;
  }
  BinaryOp(lhs, rhs, '-');
}
Num operator-(const Num lhs, const Num rhs) {
  Num num = lhs;
  num -= rhs;
  return num;
}

bool operator<(const Num lhs, const Num rhs) {
  float lhs_val = lhs.type == FLOAT ? lhs.float_value : lhs.int_value;
  float rhs_val = rhs.type == FLOAT ? rhs.float_value : rhs.int_value;
  return lhs_val < rhs_val;
}
bool operator<=(const Num lhs, const Num rhs) {
  float lhs_val = lhs.type == FLOAT ? lhs.float_value : lhs.int_value;
  float rhs_val = rhs.type == FLOAT ? rhs.float_value : rhs.int_value;
  return lhs_val <= rhs_val;
}

bool operator>(const Num lhs, const Num rhs) { return !(lhs <= rhs); }
bool operator>=(const Num lhs, const Num rhs) { return !(lhs < rhs); }

Num operator-(const Num num) {
  Num ret = num;
  ret.float_value = -ret.float_value;
  ret.int_value = -ret.int_value;
  return ret;
}

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

bool operator==(const Expression lhs, const Expression rhs) {
  if (lhs.constant != rhs.constant) return false;
  if (lhs.variable_coeff.size() != rhs.variable_coeff.size()) return false;
  for (auto entry : lhs.variable_coeff) {
    if (rhs.variable_coeff.find(entry.first) == rhs.variable_coeff.end())
      return false;
    if (entry.second != rhs.variable_coeff.find(entry.first)->second)
      return false;
  }
  return true;
}

Expression operator*(const Variable var, const Num num) {
  Expression exp(num.type == FLOAT ? kFloatZero : kIntZero);
  exp.SetCoeffOf(var, num);
  return exp;
}
Expression operator*(const Num num, const Variable var) { return var * num; }

void operator+=(Expression &lhs, const Expression rhs) {
  for (auto entry : rhs.variable_coeff) {
    Num lhs_coeff = lhs.GetCoeffOf(entry.first);
    if (lhs_coeff != kFloatZero and lhs_coeff != kIntZero) {
      lhs.SetCoeffOf(entry.first, lhs_coeff + entry.second);
    } else {
      lhs.SetCoeffOf(entry.first, entry.second);
    }
  }
  lhs.constant += rhs.constant;
}
Expression operator+(const Expression exp1, const Expression exp2) {
  Expression exp = exp1;
  exp += exp2;
  return exp;
}

void operator-=(Expression &lhs, const Expression rhs) {
  for (auto entry : rhs.variable_coeff) {
    Num lhs_coeff = lhs.GetCoeffOf(entry.first);
    if (lhs_coeff != kFloatZero and lhs_coeff != kIntZero) {
      lhs.SetCoeffOf(entry.first, lhs_coeff - entry.second);
    } else {
      lhs.SetCoeffOf(entry.first, -1.0f * entry.second);
    }
  }
  lhs.constant -= rhs.constant;
}
Expression operator-(const Expression exp1, const Expression exp2) {
  Expression exp = exp1;
  exp -= exp2;
  return exp;
}

void operator*=(Expression &exp, const Num multiplier) {
  if (multiplier == kIntZero or multiplier == kFloatZero) {
    exp = Expression(multiplier);
  }
  std::vector<Variable> to_be_cleaned;
  for (auto &entry : exp.variable_coeff) {
    entry.second *= multiplier;
    if (entry.second.IsZero()) {
      to_be_cleaned.push_back(entry.first);
    }
  }
  for (auto var : to_be_cleaned) {
    exp.variable_coeff.erase(var);
  }
  exp.constant *= multiplier;
}
Expression operator*(const Expression exp1, const Num multiplier) {
  Expression exp = exp1;
  exp *= multiplier;
  return exp;
}
Expression operator*(const Num multiplier, const Expression exp1) {
  return exp1 * multiplier;
}

void operator/=(Expression &exp, const Num multiplier) {
  std::vector<Variable> to_be_cleaned;
  for (auto &entry : exp.variable_coeff) {
    entry.second /= multiplier;
    if (entry.second.IsZero()) {
      to_be_cleaned.push_back(entry.first);
    }
  }
  for (auto var : to_be_cleaned) {
    exp.variable_coeff.erase(var);
  }
  exp.constant /= multiplier;
}
Expression operator/(const Expression exp1, const Num multiplier) {
  Expression exp = exp1;
  exp /= multiplier;
  return exp;
}

void ReplaceVariableWithExpression(Expression &expression, Variable var,
                                   Expression substitution) {
  auto coeff = expression.GetCoeffOf(var);
  expression.SetCoeffOf(var, var.type == FLOAT ? kFloatZero : kIntZero);
  expression += substitution * coeff;
}

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

bool operator==(const Constraint lhs, const Constraint rhs) {
  return (lhs.compare == rhs.compare) and (lhs.expression == rhs.expression) and
         (lhs.equation_type == rhs.equation_type) and
         (lhs.data_type == rhs.data_type);
}

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
