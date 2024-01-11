#include "ilp.h"

/* The general steps of cutting plan method:
 *    1. Convert the raw problem into relaxed form without integer constraints
 *       and solve it with normal LP solver.
 *    2. If there are non-integers in the solution, find a linear constraint and
 *       add it to the raw LP problem.
 *    3. Repeat this process until an all-integer solution is found.
 */
ILPModel::Result ILPModel::CuttingPlaneSolve() {
  LPModel model;

  for (auto constraint : constraints_) {
    constraint.SetDataType(FLOAT);
    constraint.compare.To(FLOAT);
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
    model.AddConstraint(constraint);
  }
  OptimizationObject obj(FLOAT);
  for (auto &entry : opt_obj_.expression.variable_coeff) {
    Variable var = entry.first;
    Num coeff = entry.second;
    coeff.To(FLOAT);
    var.To(FLOAT);
    obj.expression += coeff * var;
  }
  obj.opt_type = opt_obj_.opt_type;
  obj.expression.constant = opt_obj_.expression.constant;
  obj.expression.constant.To(FLOAT);
  model.SetOptimizationObject(obj);

  model.ToStandardForm();
  model.ToSlackForm();
  while (true) {
    auto result = model.Solve();
    if (result == NOSOLUTION) return NOSOLUTION;
    if (result == UNBOUNDED) return UNBOUNDED;
    bool all_intergral = true;
    for (auto &constraint : model.constraints_) {
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
  Variable slack = CreateBaseVariable();
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
