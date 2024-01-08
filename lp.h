/*
 * Created on Sun Jan 07 2023
 *
 * Copyright (c) 2024 - Qiming Zheng
 *
 * This file defines the interface of solving a linear programming problem with
 * Simplex method (https://en.wikipedia.org/wiki/Simplex_algorithm).
 *
 */
#pragma once

#include <assert.h>

#include "base.h"

const std::string kBase = "base";
const std::string kSubstitution = "subst";

bool IsUserDefined(Variable var) {
  return var.variable_name.rfind(kBase, 0) != 0 &&
         var.variable_name.rfind(kSubstitution, 0) != 0;
}

class LPModel {
 public:
  enum Result {
    UNBOUNDED,
    NOSOLUTION,
    SOLVED,
  };

  LPModel() : opt_obj_(OptimizationObject(FLOAT)) { Reset(); }

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

  friend class ILPModel;

  friend bool StandardFormSanityCheck(LPModel model);
  friend bool SlackFormSanityCheck(LPModel model);

 private:
  std::vector<Constraint> constraints_;
  OptimizationObject opt_obj_;
  std::set<Variable> base_variables_;
  std::set<Variable> non_base_variables_;
  bool opt_reverted_ = false;
  // The only purpose of this field is used to do sanity check after the
  // standard form transformation.
  std::set<Variable> non_negative_variables_;
};

Variable CreateBaseVariable();

Variable CreateSubstitutionVariable();

bool StandardFormSanityCheck(LPModel model);

bool SlackFormSanityCheck(LPModel model);
