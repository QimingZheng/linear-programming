#pragma once

#include <assert.h>

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

typedef float Num;

const std::string kUndefined = "undefined";
const std::string kBase = "base";
const std::string kSubstitution = "subst";

struct Variable {
 public:
  Variable() { variable_name = kUndefined; }
  Variable(std::string variable_name) { this->variable_name = variable_name; }

  bool IsUndefined() const { return variable_name == kUndefined; }

  static Variable CreateBaseVariable() {
    auto var = Variable(kBase + std::to_string(base_variable_count_));
    base_variable_count_ += 1;
    return var;
  }

  static Variable CreateSubstitutionVariable() {
    auto var =
        Variable(kSubstitution + std::to_string(substitution_variable_count_));
    substitution_variable_count_ += 1;
    return var;
  }

  static void Reset() {
    base_variable_count_ = 0;
    substitution_variable_count_ = 0;
  }

  std::string variable_name;

  static int base_variable_count_;
  static int substitution_variable_count_;
};

bool operator==(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name == rhs.variable_name;
}

bool operator!=(const Variable &lhs, const Variable &rhs) {
  return lhs.variable_name != rhs.variable_name;
}

bool operator<(const Variable &lhs, const Variable &rhs) {
  // TODO: if start with kBase, then should be of lower priority.
  return lhs.variable_name < rhs.variable_name;
}

struct Expression {
 public:
  void SetConstant(Num constant) { this->constant = constant; }
  void Multiply(Num multiplier) {
    for (auto &entry : variable_coeff) {
      entry.second *= multiplier;
    }
    constant *= multiplier;
  }
  void Add(Expression expression);

  Num GetCoeffOf(Variable var) {
    if (variable_coeff.find(var) == variable_coeff.end())
      return 0.0;
    else
      return variable_coeff[var];
  }
  void SetCoeffOf(Variable var, Num num) {
    if (num == 0.0) {
      variable_coeff.erase(var);
    } else {
      variable_coeff[var] = num;
    }
  }

  void ReplaceVariableWithExpression(Variable var, Expression expression);

  std::string ToString() {
    std::string ret = "";
    int i = 0;
    for (auto entry : variable_coeff) {
      i += 1;
      ret += std::to_string(entry.second) + " * " + entry.first.variable_name +
             (i == variable_coeff.size() ? "" : " + ");
    }
    if (constant != 0.0)
      ret += (ret.length() ? " + " : "") + std::to_string(constant);
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

  void AddItem(Num coeff, Variable var) { expression.SetCoeffOf(var, coeff); }
  void SetConstant(Num constant) { expression.SetConstant(constant); }
  void SetCompare(Num compare) { this->compare = compare; }
  void SetType(Type type) { this->type = type; }

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
    ret += " " + std::to_string(compare);
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

  void AddItem(Num coeff, Variable var) { expression.SetCoeffOf(var, coeff); }
  void SetType(Type type) { this->type = type; }

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
    return ret + expression.ToString();
  }

  Expression expression;
  Type type = MIN;
};

class LPModel {
 public:
  enum Result {
    UNBOUNDED,
    NOSOLUTION,
    SOLVED,
  };

  LPModel() { Variable::Reset(); }

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
  void ToStandardForm();

  // Transform the LP model to the relaxed form:
  void ToRelaxedForm();

  Result Solve();

  Num GetOptimum();

  std::string ToString() {
    std::string ret = "";
    ret += opt_obj_.ToString() + "\n";
    for (auto constraint : constraints_) {
      ret += constraint.ToString() + "\n";
    }
    return ret;
  }

  void Pivot(Variable base, Variable non_base);

 private:
  std::vector<Constraint> constraints_;
  OptimizationObject opt_obj_;
  std::set<Variable> base_variables_;
  std::set<Variable> non_base_variables_;
  bool opt_reverted_ = false;
};
