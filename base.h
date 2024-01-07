#pragma once

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

enum DataType {
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