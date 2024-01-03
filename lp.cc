#include "lp.h"

void Expression::ReplaceVariableWithExpression(Variable var,
                                               Expression replace) {
  Num coeff = variable_coeff[var];
  constant += coeff * replace.constant;
  variable_coeff.erase(var);
  for (auto entry : replace.variable_coeff) {
    if (variable_coeff.find(entry.first) != variable_coeff.end()) {
      variable_coeff[entry.first] += coeff * entry.second;
    } else {
      variable_coeff[entry.first] = coeff * entry.second;
    }
  }
  std::vector<Variable> to_clean_up;
  for (auto entry : variable_coeff) {
    if (entry.second == 0.0) {
      to_clean_up.push_back(entry.first);
    }
  }
  for (auto var : to_clean_up) {
    variable_coeff.erase(var);
  }
}

void LPModel::Pivot(Variable base, Variable non_base) {
  Expression substitution;
  for (auto& constraint : constraints_) {
    if (constraint.expression.variable_coeff.find(base) !=
            constraint.expression.variable_coeff.end() and
        constraint.expression.variable_coeff.find(non_base) !=
            constraint.expression.variable_coeff.end()) {
      Num dividend = (-constraint.expression.variable_coeff[non_base]);
      substitution.SetConstant(constraint.expression.constant / dividend);
      for (auto entry : constraint.expression.variable_coeff) {
        if (entry.first != non_base)
          substitution.AddItem(entry.second / dividend, entry.first);
      }
      base_variables_.erase(base);
      base_variables_.insert(non_base);
      non_base_variables_.erase(non_base);
      non_base_variables_.insert(base);
      break;
    }
  }
  for (auto& constraint : constraints_) {
    if (constraint.expression.variable_coeff.find(base) ==
            constraint.expression.variable_coeff.end() and
        constraint.expression.variable_coeff.find(non_base) !=
            constraint.expression.variable_coeff.end()) {
      constraint.expression.ReplaceVariableWithExpression(non_base,
                                                          substitution);
    }
  }
}

void LPModel::Solve() {
  // Step 1: If there are negative constants in some constraint, use Pivot to
  // convert them to positive.

  // Step 2: The main optimization procedures.
  while (true) {
    Variable e;
    for (auto& entry : opt_obj_.expression.variable_coeff) {
      if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
          entry.second > 0) {
        e = entry.first;
        break;
      }
    }
    if (e.variable_name == "undefined") {
      return;
    }
    Variable d;
    Num min_ = -10000000;
    for (auto var : base_variables_) {
      for (auto constraint : constraints_) {
        if (constraint.expression.variable_coeff.find(var) !=
            constraint.expression.variable_coeff.end()) {
          if (-constraint.expression.variable_coeff[var] > 0) {
            if (min_ > constraint.expression.constant /
                           (-constraint.expression.variable_coeff[var])) {
              min_ = constraint.expression.constant /
                     (-constraint.expression.variable_coeff[var]);
              d = var;
            }
          }
        }
      }
    }
    if (d.variable_name == "undefined") {
      return;
    }
    Pivot(e, d);
  }
}
