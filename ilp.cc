#include "ilp.h"

Result ILPModel::BranchAndBoundSolve() {
  std::queue<ILPModel> problems;

  Result result = NOSOLUTION;
  Num optimal(-100000000.f);
  std::map<Variable, Num> sol;
  problems.push(*this);
  auto is_integral = [](float x) -> bool { return fabs(x - int(x)) < 1e-6; };
  std::set<Variable> interger_vars;
  for (auto constraint : model_.constraints) {
    for (auto entry : constraint.expression.variable_coeff)
      interger_vars.insert(entry.first);
  }
  auto is_integral_solution = [&](std::map<Variable, Num> solution) -> bool {
    for (auto entry : solution) {
      auto var = entry.first;
      var.To(INTEGER);
      if (interger_vars.find(var) == interger_vars.end()) continue;
      if (!is_integral(entry.second.float_value)) {
        return false;
      }
    }
    return true;
  };
  while (!problems.empty()) {
    auto sub_problem = problems.front();
    problems.pop();
    auto relaxed_sub_problem = sub_problem.ToRelaxedLPModel();
    auto sub_result = relaxed_sub_problem.Solve();
    if (sub_result == NOSOLUTION) continue;
    if (sub_result == UNBOUNDED) return UNBOUNDED;
    if (relaxed_sub_problem.GetOptimum() < optimal) continue;
    if (is_integral_solution(relaxed_sub_problem.GetSolution())) {
      optimal = relaxed_sub_problem.GetOptimum();
      sol = relaxed_sub_problem.GetSolution();
      result = SOLVED;
      continue;
    }
    // The optimal solution of this sub problem is better than the optimal so
    // far, need to split it into more sub-problems.
    for (auto entry : relaxed_sub_problem.GetSolution()) {
      if (!is_integral(entry.second.float_value)) {
        auto var = entry.first;
        var.To(INTEGER);
        Constraint c1(INTEGER), c2(INTEGER);
        c1.expression = var - Num(int(entry.second.float_value));
        c1.equation_type = Constraint::Type::LE;
        c2.expression = var - Num(int(entry.second.float_value) + 1);
        c2.equation_type = Constraint::Type::GE;
        auto sub_problem_1 = sub_problem, sub_problem_2 = sub_problem;
        sub_problem_1.AddConstraint(c1);
        sub_problem_2.AddConstraint(c2);
        problems.push(sub_problem_1);
        problems.push(sub_problem_2);
        break;
      }
    }
  }
  optimum_ = optimal;
  for (auto entry : sol) {
    auto var = entry.first;
    var.To(INTEGER);
    solution_[var] = entry.second;
  }
  return result;
}

/* The general steps of cutting plane method:
 *    1. Convert the raw problem into relaxed form without integer constraints
 *       and solve it with normal LP solver.
 *    2. If there are non-integers in the solution, find a linear constraint and
 *       add it to the raw LP problem.
 *    3. Repeat this process until an all-integer solution is found.
 */
Result ILPModel::CuttingPlaneSolve() {
  LPModel model = ToRelaxedLPModel();
  while (true) {
    auto result = model.Solve();
    if (result == NOSOLUTION) return NOSOLUTION;
    if (result == UNBOUNDED) return UNBOUNDED;
    bool all_intergral = true;
    for (auto &constraint : model.model_.constraints) {
      float b = constraint.expression.constant.float_value;
      // If b < 0, int(b) will be larger than b.
      int b_interger_part = b < 0.0f ? int(b) - 1 : int(b);
      int b_decimal_part = b - b_interger_part;
      if (abs(b_decimal_part) > 1e-6) {
        all_intergral = false;
        model.AddConstraint(FindGomoryCut(model, constraint));
        break;
      }
    }
    if (all_intergral) {
      optimum_ = model.GetOptimum();
      solution_ = model.GetSolution();
      break;
    }
  }
  return SOLVED;
}

// Gomory's cut
Constraint ILPModel::FindGomoryCut(
    LPModel &model, Constraint non_integral_variable_constraint) {
  float b = non_integral_variable_constraint.expression.constant.float_value;
  // If b < 0, int(b) will be larger than b.
  int b_interger_part = b < 0.0f ? int(b) - 1 : int(b);
  int b_decimal_part = b - b_interger_part;
  Constraint con(FLOAT);
  con.equation_type = Constraint::Type::EQ;
  con.compare = kFloatZero;
  con.expression.constant = b_interger_part - b;
  Variable slack = model.CreateBaseVariable();
  model.AddBaseVariable(slack);
  con.expression -= slack;
  for (auto &entry :
       non_integral_variable_constraint.expression.variable_coeff) {
    if (model.IsBaseVariable(entry.first)) continue;
    auto coeff = -entry.second;
    auto coeff_integer_part =
        Num(coeff.float_value < 0.0f ? int(coeff.float_value) - 1
                                     : int(coeff.float_value));
    con.expression -= (coeff_integer_part - coeff) * entry.first;
  }
  return con;
}

// Convert the raw problem into the relaxed linear programming model.
LPModel ILPModel::ToRelaxedLPModel() {
  LPModel model;

  for (auto constraint : model_.constraints) {
    auto con = Constraint(FLOAT);
    con.equation_type = constraint.equation_type;
    con.compare = constraint.compare;
    con.compare.To(FLOAT);
    Expression exp(FLOAT);
    for (auto &entry : constraint.expression.variable_coeff) {
      Variable var = entry.first;
      Num coeff = entry.second;
      coeff.To(FLOAT);
      var.To(FLOAT);
      exp += coeff * var;
    }
    exp.constant = constraint.expression.constant;
    exp.constant.To(FLOAT);
    con.expression = exp;
    model.AddConstraint(con);
  }
  OptimizationObject obj(FLOAT);
  for (auto &entry : model_.opt_obj.expression.variable_coeff) {
    Variable var = entry.first;
    Num coeff = entry.second;
    coeff.To(FLOAT);
    var.To(FLOAT);
    obj.expression += coeff * var;
  }
  obj.opt_type = model_.opt_obj.opt_type;
  obj.expression.constant = model_.opt_obj.expression.constant;
  obj.expression.constant.To(FLOAT);
  model.SetOptimizationObject(obj);

  model.ToStandardForm();
  model.ToSlackForm();
  return model;
}
