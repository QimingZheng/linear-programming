/*
 * Created on Sun Jan 07 2024
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

struct Model {
  std::vector<Constraint> constraints;
  OptimizationObject opt_obj;
};

enum Result {
  ERROR,
  UNBOUNDED,
  NOSOLUTION,
  SOLVED,
};

class LPModel {
 public:
  LPModel(Model model) : model_(model) {}
  LPModel() : model_({{}, OptimizationObject(FLOAT)}) {}

  void AddConstraint(Constraint constraint) {
    model_.constraints.push_back(constraint);
  }
  void SetOptimizationObject(OptimizationObject obj) {
    assert(obj.expression.constant == kFloatZero);
    model_.opt_obj = obj;
  }

  void SetEnableLogging(bool enable_logging) {
    enable_logging_ = enable_logging;
  }
  void SetLogEvery(int iters) {
    assert(iters > 0);
    log_every_iters_ = iters;
  }

  // Transform the LP model to standard form:
  //  1. optimization object: maximization
  //  2. all constraints have the following form:
  //      \sum_{i} c_i x_i <= b
  void ToStandardForm();

  // Transform the LP model to the slack form.
  void ToSlackForm();

  /* The Simplex Method. See: https://en.wikipedia.org/wiki/Simplex_algorithm */
  // The key operation of the simplex method.
  void Pivot(Variable base, Variable non_base);

  // The phase 1 of the simplex method: initialization.
  Result Initialize();

  // The phase 2 (main step) of the simplex method.
  Result SimplexSolve();

  Num GetSimplexOptimum();

  std::map<Variable, Num> GetSimplexSolution();

  // The extreme ray is an unbounded direction.
  std::map<Variable, Num> GetSimplexExtremeRay();

  /* The Dual Simplex Method. */
  Result DualSolve(std::set<Variable> dual_feasible_solution_basis);

  Num GetDualSolveOptimum();

  std::map<Variable, Num> GetDualSolveSolution();

  // Solves the linear programming problem with column generation algorithm:
  // https://en.wikipedia.org/wiki/Column_generation
  Result ColumnGenerationSolve(std::set<Variable> initial_solution_basis = {},
                               bool initialize_solution_with_two_phase = false);

  void ColumnGenerationInitializeSolutionWithBigM(
      LPModel& master_problem, std::set<Variable>& artificials);

  void ColumnGenerationInitializeSolutionWithTwoPhase(
      LPModel& master_problem, std::set<Variable>& artificials);

  Num GetColumnGenerationOptimum();

  std::map<Variable, Num> GetColumnGenerationSolution();

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

  std::string ToString() {
    std::string ret = "";
    ret += model_.opt_obj.ToString() + "\n";
    for (auto constraint : model_.constraints) {
      ret += constraint.ToString() + "\n";
    }
    return ret;
  }

  Variable CreateBaseVariable();

  Variable CreateSubstitutionVariable();

  Variable CreateDualVariable();

  Variable CreateArtificialVariable();

 private:
  void LogIterStatus(int iter, long delta, real_t optimum);
  // Check if the constraint is in the form of: x >= 0
  bool IsNonNegativeConstraint(const Constraint& constraint);

  // Suppose the current LP form is the optimal solution, which means:
  // the coefficients of all non-base variables in the objective function
  // are non-negative. `GetOptimum()`, `GetSolution()` should return its optimal
  // solution accordingly.
  Num GetOptimum(bool check_optimal_condition = false);
  std::map<Variable, Num> GetSolution();
  std::map<Variable, Num> GetRay(Variable non_basis_var);

  Model model_;
  std::set<Variable> base_variables_;
  std::set<Variable> non_base_variables_;
  bool opt_reverted_ = false;
  // The only purpose of this field is used to do sanity check after the
  // standard form transformation.
  std::set<Variable> non_negative_variables_;
  std::map<Variable, Expression> raw_variable_expression_;

  // Variables that are overrided as user defined vars (usually used in method
  // ToDualForm).
  std::set<Variable> overrided_as_user_defined_variables_;
  void OverrideAsUserDefined(Variable var);
  bool IsOverriddenAsUserDefined(Variable var);

  // The solution of simplex method.
  Num simplex_optimum_;
  std::map<Variable, Num> simplex_solution_;
  std::map<Variable, Num> simplex_extreme_ray_;

  // The solution of dual simplex method.
  Num dual_simplex_optimum_;
  std::map<Variable, Num> dual_simplex_solution_;

  // The solution of column generation method.
  Num column_generation_optimum_;
  std::map<Variable, Num> column_generation_solution_;

  int base_variable_count_ = 0;
  int substitution_variable_count_ = 0;
  int dual_variable_count_ = 0;
  int artificial_variable_count_ = 0;

  bool enable_logging_ = false;
  int log_every_iters_ = 1;
};

bool StandardFormSanityCheck(LPModel model);

bool SlackFormSanityCheck(LPModel model);
