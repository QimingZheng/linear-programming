#include "lp.h"

void LPModel::ColumnGenerationInitializeSolutionWithBigM(
    LPModel& master_problem, std::set<Variable>& artificials) {
  // Initialize the initial master problem by adding artificial variables to set
  // up the initial restricted master problem:
  //    max c^T x - \infinity^T y
  //    s.t. Ax +/- y <= b (`+` if (b >= 0) else `-`)
  //      x, y >= 0
  // Then a trivial feasible solution to the raw problem is: x = 0, y = |b|
  for (size_t i = 0; i < model_.constraints.size(); i++) {
    auto artificial = CreateArtificialVariable();
    artificials.insert(artificial);
    non_base_variables_.insert(artificial);
    if (model_.constraints[i].compare.IsNonNegative()) {
      model_.constraints[i].expression += 1.0f * artificial;
    } else {
      model_.constraints[i].expression += -1.0f * artificial;
    }
    Constraint constraint(FLOAT);
    constraint.equation_type = Constraint::LE;
    constraint.expression =
        model_.constraints[i].expression.GetCoeffOf(artificial) * artificial;
    constraint.compare = model_.constraints[i].compare;
    master_problem.AddConstraint(constraint);
  }
  master_problem.opt_reverted_ = opt_reverted_;
  OptimizationObject opt(FLOAT);
  opt.opt_type = OptimizationObject::MAX;
  for (auto artificial : artificials) {
    model_.opt_obj.expression += -1000000000.0f * artificial;
    opt.expression += -1000000000.0f * artificial;
  }
  master_problem.SetOptimizationObject(opt);
  std::set<Variable> added_variables = artificials;
  master_problem.non_base_variables_ = artificials;
  master_problem.non_negative_variables_ = artificials;
}

void LPModel::ColumnGenerationInitializeSolutionWithTwoPhase(
    LPModel& master_problem, std::set<Variable>& initial_solution_basis) {
  LPModel phase_one_model = *this;
  auto artificial = CreateArtificialVariable();
  auto opt = OptimizationObject(FLOAT);
  opt.SetOptType(OptimizationObject::MAX);
  opt.expression = -1.0 * artificial;
  phase_one_model.SetOptimizationObject(opt);
  for (size_t i = 0; i < phase_one_model.model_.constraints.size(); i++) {
    phase_one_model.model_.constraints[i].expression += -1.0f * artificial;
  }
  assert(phase_one_model.ColumnGenerationSolve({artificial}, false) == SOLVED);

  assert(phase_one_model.GetColumnGenerationOptimum().IsZero());
  for (auto entry : phase_one_model.GetColumnGenerationSolution()) {
    if (!entry.second.IsZero()) {
      initial_solution_basis.insert(entry.first);
    }
  }
  // special case: \vec 0 is a feasible solution to the phase 1 problem.
  if (initial_solution_basis.size() == 0) {
    for (auto entry : phase_one_model.GetColumnGenerationSolution()) {
      if (entry.first != artificial) {
        initial_solution_basis.insert(entry.first);
        break;
      }
    }
  }
}

Result LPModel::ColumnGenerationSolve(std::set<Variable> initial_solution_basis,
                                      bool initialize_solution_with_two_phase) {
  LPModel master_problem;
  std::set<Variable> artificials;
  std::set<Variable> added_variables;
  if (initial_solution_basis.size() == 0) {
    if (initialize_solution_with_two_phase) {
      ColumnGenerationInitializeSolutionWithTwoPhase(master_problem,
                                                     initial_solution_basis);
    }
  }
  if (initial_solution_basis.size() != 0) {
    added_variables = initial_solution_basis;
    OptimizationObject opt = model_.opt_obj;
    for (auto entry : model_.opt_obj.expression.variable_coeff) {
      if (initial_solution_basis.find(entry.first) ==
          initial_solution_basis.end())
        opt.expression.SetCoeffOf(entry.first, 0);
    }
    master_problem.SetOptimizationObject(opt);
    for (auto constraint : model_.constraints) {
      Constraint con = constraint;
      for (auto entry : constraint.expression.variable_coeff) {
        if (initial_solution_basis.find(entry.first) ==
            initial_solution_basis.end()) {
          con.expression.SetCoeffOf(entry.first, 0);
        }
      }
      master_problem.AddConstraint(con);
    }
    master_problem.opt_reverted_ = opt_reverted_;
    master_problem.non_base_variables_ = initial_solution_basis;
    master_problem.non_negative_variables_ = initial_solution_basis;
  } else {
    if (!initialize_solution_with_two_phase) {
      ColumnGenerationInitializeSolutionWithBigM(master_problem, artificials);
      added_variables = artificials;
    }
  }
  while (true) {
    // Step 1: solve the dual problem:
    LPModel dual_problem = master_problem.ToDualForm();
    dual_problem.ToStandardForm();
    dual_problem.ToSlackForm();
    auto result = dual_problem.SimplexSolve();
    if (result == NOSOLUTION) return UNBOUNDED;
    if (result == UNBOUNDED) {
      return NOSOLUTION;
    }
    assert(result == SOLVED);
    auto sol = dual_problem.GetSimplexSolution();
    // Pricing problem: Find a non-base x_{j} with maximized reduced cost,
    // where the pricing objective function is: c_{j} - u^T b (u is the solution
    // to the dual LP).
    std::set<Variable> all_vars = non_base_variables_;
    Variable to_be_added;
    Num val = Num(-1000000000.f);
    for (auto var : all_vars) {
      if (added_variables.find(var) != added_variables.end()) continue;
      auto c = model_.opt_obj.expression.GetCoeffOf(var);
      Num u = kFloatZero;
      int i = 0;
      for (auto entry : sol) {
        u += model_.constraints[i].expression.GetCoeffOf(var) * entry.second;
        i += 1;
      }
      if (val < c - u) {
        val = c - u;
        to_be_added = var;
      }
    }
    // If no improving variables can be found, then the optimum is achieved.
    if (to_be_added.IsUndefined()) break;
    // Add the improving variable to the master problem (both objective function
    // and constraints).
    for (size_t i = 0; i < model_.constraints.size(); i++) {
      auto con = model_.constraints[i];
      master_problem.model_.constraints[i].expression +=
          con.expression.GetCoeffOf(to_be_added) * to_be_added;
    }
    master_problem.model_.opt_obj.expression +=
        model_.opt_obj.expression.GetCoeffOf(to_be_added) * to_be_added;
    master_problem.non_base_variables_.insert(to_be_added);
    master_problem.non_negative_variables_.insert(to_be_added);
    added_variables.insert(to_be_added);
  }
  if (initial_solution_basis.size() == 0) {
    if (!initialize_solution_with_two_phase) {
      for (auto art : artificials) {
        master_problem.model_.opt_obj.expression.SetCoeffOf(art, kFloatZero);
        for (auto& con : master_problem.model_.constraints) {
          con.expression.SetCoeffOf(art, kFloatZero);
        }
      }
    }
  }
  master_problem.ToSlackForm();
  master_problem.SimplexSolve();
  column_generation_optimum_ = master_problem.GetSimplexOptimum();
  column_generation_solution_ = master_problem.GetSimplexSolution();
  return SOLVED;
}

Num LPModel::GetColumnGenerationOptimum() { return column_generation_optimum_; }

std::map<Variable, Num> LPModel::GetColumnGenerationSolution() {
  return column_generation_solution_;
}