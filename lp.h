#pragma once

#include <assert.h>

#include <iostream>
#include <map>
#include <set>
#include <string>
typedef float Num;

struct Variable {
 public:
  Variable() { variable_name = "undefined"; }
  Variable(std::string variable_name) { this->variable_name = variable_name; }

  std::string variable_name;
};

bool operator==(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name == rhs.variable_name;
}

bool operator!=(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name != rhs.variable_name;
}

bool operator<(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name < rhs.variable_name;
}

struct Expression {
 public:
  void AddItem(Num coeff, Variable var) {
    if (variable_coeff.find(var) != variable_coeff.end()) return;
    if (coeff == 0.0) return;
    variable_coeff[var] = coeff;
  }
  void SetConstant(Num constant) { this->constant = constant; }

  void ReplaceVariableWithExpression(Variable var, Expression expression);

  std::string ToString() {
    std::string ret = "";
    for (auto entry : variable_coeff) {
      ret += std::to_string(entry.second) + " * " + entry.first.variable_name +
             " + ";
    }
    ret += std::to_string(constant);
    return ret;
  }

  std::map<Variable, Num> variable_coeff;
  Num constant = 0.0;
};

// \sum_{i} c_i * x_i + constant <=/>=/= compare.
struct Constraint {
 public:
  enum Type {
    LE,  // less than or equal to
    GE,  // greater than or equal to
    EQ,  // equal to
  };

  void AddItem(Num coeff, Variable var);
  void AddConstant(Num constant);
  void SetCompare(Num compare);
  void SetType(Type type);
  void Replace();

  std::string ToString() {
    std::string ret = expression.ToString() + " ";
    switch (type) {
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
    ret += " " + std::to_string(compare) + "\n";
    return ret;
  }

  Expression expression;
  Num compare = 0.0;
  Type type = Type::EQ;
};

struct OptimizationObject {
 public:
  enum Type {
    MIN,
    MAX,
  };

  std::string ToString() {
    std::string ret = "";
    switch (type) {
      case MIN:
        ret = "min ";
        break;
      case MAX:
        ret = "max ";
        break;

      default:
        break;
    }
    for (auto entry : expression.variable_coeff) {
      ret += std::to_string(entry.second) + " * " + entry.first.variable_name +
             " + ";
    }
    return ret;
  }

  Expression expression;
  Type type;
};

class LPModel {
 public:
  void AddConstraint(Constraint constraint) {
    constraints_.push_back(constraint);
  }
  void SetOptimizationObject(OptimizationObject obj) {
    assert(obj.expression.constant == 0.0);
    opt_obj_ = obj;
  }

  // Transform the LP model to standard form:
  //  1. optimization object: maximization
  //  2. all constraints have the following form:
  //      \sum_{i} c_i x_i <= b
  void ToStandardForm() {
    // Opt obj transform.
    if (opt_obj_.type == OptimizationObject::Type::MIN) {
      for (auto &entry : opt_obj_.expression.variable_coeff) {
        entry.second *= -1.0;
      }
      opt_obj_.type = OptimizationObject::Type::MAX;
    }
    for (auto &constraint : constraints_) {
      constraint.compare -= constraint.expression.constant;
      constraint.expression.constant = 0.0;
      if (constraint.type == Constraint::Type::GE) {
        for (auto &entry : constraint.expression.variable_coeff) {
          entry.second *= -1.0;
        }
        constraint.compare *= -1.0;
        constraint.type = Constraint::Type::LE;
      } else if (constraint.type == Constraint::Type::EQ) {
        // TODO: split this constraint into two constraints.
      }
    }
    // TODO: add constraints for un-constrained variables.
  }

  // Transform the LP model to the relaxed form:
  void ToRelaxedForm() {
    for (auto &constraint : constraints_) {
      auto base_var = Variable();
      for (auto &entry : constraint.expression.variable_coeff) {
        entry.second *= -1.0;
      }
      constraint.expression.constant = constraint.compare;
      constraint.compare = 0.0;
      constraint.type = Constraint::Type::EQ;
      constraint.AddItem(-1.0, base_var);
      base_variables_.insert(base_var);
    }
  }

  void Solve();

  std::string ToString() {
    opt_obj_.ToString();
    for (auto constraint : constraints_) {
      constraint.ToString();
    }
  }

 private:
  void Pivot(Variable base, Variable non_base);
  std::vector<Constraint> constraints_;
  OptimizationObject opt_obj_;
  std::set<Variable> base_variables_;
  std::set<Variable> non_base_variables_;
};
