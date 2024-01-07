#pragma once

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "base.h"

const std::string kBase = "base";
const std::string kSubstitution = "subst";

bool IsUserDefined(Variable var) {
  return var.variable_name.rfind(kBase, 0) != 0 &&
         var.variable_name.rfind(kSubstitution, 0) != 0;
}

// \sum_{i} c_i * x_i + constant <=/>=/= compare.
struct Constraint {
 public:
  enum Type {
    LE,  // less than or equal to
    GE,  // greater than or equal to
    EQ,  // equal to
  };

  void SetConstant(Num constant) { expression.constant = constant; }
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
    ret += " " + compare.ToString();
    return ret;
  }

  Expression expression = Expression(0.0f);
  Num compare = 0.0f;
  Type type = Type::EQ;
};

struct OptimizationObject {
 public:
  enum Type {
    MIN,
    MAX,
  };

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

  Expression expression = Expression(0.0f);
  Type type = MIN;
};

class LPModel {
 public:
  enum Result {
    UNBOUNDED,
    NOSOLUTION,
    SOLVED,
  };

  LPModel() { Reset(); }

  void AddConstraint(Constraint constraint) {
    constraints_.push_back(constraint);
  }
  void SetOptimizationObject(OptimizationObject obj) {
    assert(obj.expression.constant == 0.0f);
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

  std::map<Variable, Num> GetSolution();

  std::string ToString() {
    std::string ret = "";
    ret += opt_obj_.ToString() + "\n";
    for (auto constraint : constraints_) {
      ret += constraint.ToString() + "\n";
    }
    return ret;
  }

  void Pivot(Variable base, Variable non_base);

  Result Initialize();

  static int base_variable_count_;
  static int substitution_variable_count_;

  void Reset() {
    LPModel::base_variable_count_ = 0;
    LPModel::substitution_variable_count_ = 0;
  }

 private:
  std::vector<Constraint> constraints_;
  OptimizationObject opt_obj_;
  std::set<Variable> base_variables_;
  std::set<Variable> non_base_variables_;
  bool opt_reverted_ = false;
};

Variable CreateBaseVariable();

Variable CreateSubstitutionVariable();