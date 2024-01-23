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
const std::string kDual = "dual";
const std::string kArtificial = "artificial";

bool IsUserDefined(Variable var);

enum Result {
  UNBOUNDED,
  NOSOLUTION,
  SOLVED,
};

class LPModel {
 public:
  LPModel() : opt_obj_(OptimizationObject(FLOAT)) { Reset(); }

  void AddConstraint(Constraint constraint) {
    constraints_.push_back(constraint);
  }
  void SetOptimizationObject(OptimizationObject obj) {
    assert(obj.expression.constant == kFloatZero);
    opt_obj_ = obj;
  }

  // Transform the LP model to standard form:
  //  1. optimization object: maximization
  //  2. all constraints have the following form:
  //      \sum_{i} c_i x_i <= b
  void ToStandardForm();

  // Transform the LP model to the slack form.
  void ToSlackForm();

  Result Solve();

  Result DualSolve(std::set<Variable> dual_feasible_solution_basis);

  // Solves the linear programming problem with column generation algorithm:
  // https://en.wikipedia.org/wiki/Column_generation
  Result ColumnGenerationSolve();

  Num GetColumnGenerationOptimum();

  std::map<Variable, Num> GetColumnGenerationSolution();

  Num GetOptimum();

  Num GetDualSolveOptimum();

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
  static int dual_variable_count_;
  static int artificial_variable_count_;

  void Reset() {
    LPModel::base_variable_count_ = 0;
    LPModel::substitution_variable_count_ = 0;
    LPModel::dual_variable_count_ = 0;
    LPModel::artificial_variable_count_ = 0;
  }

  // Mark as virtual for the convenience of testing.
  virtual bool IsBaseVariable(Variable var) {
    return base_variables_.find(var) != base_variables_.end();
  }
  std::set<Variable> GetBaseVariables() { return base_variables_; }
  // Mark as virtual for the convenience of testing.
  virtual void AddBaseVariable(Variable var) { base_variables_.insert(var); }

  friend class ILPModel;

  friend bool StandardFormSanityCheck(LPModel model);
  friend bool SlackFormSanityCheck(LPModel model);

  std::map<Variable, Expression> GetRawVariableExpression() {
    return raw_variable_expression_;
  }

  // Transform the model to its dual form:
  // https://en.wikipedia.org/wiki/Dual_linear_program#Form_of_the_dual_LP
  LPModel ToDualForm();

  // Perform gaussian elimination on the slack form, and make the coefficients
  // of the base-variables forms an identity matrix.
  void GaussianElimination(std::set<Variable> base_variables);

 private:
  // Check if the constraint is in the form of: x >= 0
  bool IsNonNegativeConstraint(const Constraint &constraint);
  std::vector<Constraint> constraints_;
  OptimizationObject opt_obj_;
  std::set<Variable> base_variables_;
  std::set<Variable> non_base_variables_;
  bool opt_reverted_ = false;
  // The only purpose of this field is used to do sanity check after the
  // standard form transformation.
  std::set<Variable> non_negative_variables_;
  std::map<Variable, Expression> raw_variable_expression_;

  Num column_generation_optimum_;
  std::map<Variable, Num> column_generation_solution_;
};

Variable CreateBaseVariable();

Variable CreateSubstitutionVariable();

Variable CreateDualVariable();

Variable CreateArtificialVariable();

bool StandardFormSanityCheck(LPModel model);

bool SlackFormSanityCheck(LPModel model);
