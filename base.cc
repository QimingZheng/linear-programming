#include "base.h"

bool operator==(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name == rhs.variable_name and lhs.type == rhs.type;
}

bool operator!=(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name != rhs.variable_name or lhs.type != rhs.type;
}

bool operator<(const Variable &lhs, const Variable &rhs) {
  if (lhs.type != rhs.type) return lhs.type < rhs.type;
  return lhs.variable_name < rhs.variable_name;
}

bool operator==(const Num &lhs, const Num &rhs) {
  if (lhs.type != rhs.type) return false;
  auto type = lhs.type;
  switch (type) {
    case FLOAT:
      return lhs.float_value == rhs.float_value;

    case INTEGER:
      return lhs.int_value == rhs.int_value;

    default:
      throw std::runtime_error("Num == Num error\n");
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

bool operator!=(const Expression lhs, const Expression rhs) {
  return !(lhs == rhs);
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

bool operator==(const Constraint lhs, const Constraint rhs) {
  return (lhs.compare == rhs.compare) and (lhs.expression == rhs.expression) and
         (lhs.equation_type == rhs.equation_type) and
         (lhs.data_type == rhs.data_type);
}