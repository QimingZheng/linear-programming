#include "ilp.h"

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
  obj.expression.constant = opt_obj_.expression.constant;
  obj.expression.constant.To(FLOAT);
  model.SetOptimizationObject(obj);

  model.ToStandardForm();
  model.ToSlackForm();
  while (true) {
    auto result = model.Solve();
    if (result == NOSOLUTION) return NOSOLUTION;
    if (result == UNBOUNDED) return UNBOUNDED;
    // Gomory's cut
    bool all_intergral = true;
    for (auto &constraint : model.constraints_) {
      float b = constraint.expression.constant.float_value;
      int b_interger_part = int(b);
      int b_decimal_part = b - b_interger_part;
      if (abs(b_decimal_part) > 1e-6) {
        all_intergral = false;
        Constraint con(FLOAT);
        con.expression.constant = b_interger_part - b;
        Variable slack = CreateBaseVariable();
        model.base_variables_.insert(slack);
        con.expression -= slack;
        for (auto &entry : constraint.expression.variable_coeff) {
          auto coeff = -entry.second;
          auto coeff_integer_part = Num(int(coeff.float_value));
          con.expression -= (coeff_integer_part - coeff) * entry.first;
        }
        model.AddConstraint(con);
        break;
      }
    }
    if (all_intergral) {
      break;
    }
  }
  return SOLVED;
}