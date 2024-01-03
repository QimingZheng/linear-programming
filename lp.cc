#include "lp.h"

int Variable::base_variable_count_ = 0;

void Expression::Add(Expression expression) {
  constant += expression.constant;
  for (auto entry : expression.variable_coeff) {
    if (GetCoeffOf(entry.first) != 0.0) {
      variable_coeff[entry.first] += entry.second;
    } else {
      variable_coeff[entry.first] = entry.second;
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

void Expression::ReplaceVariableWithExpression(Variable var,
                                               Expression replace) {
  Num coeff = GetCoeffOf(var);
  if (coeff == 0.0) return;
  replace.Multiply(coeff);
  variable_coeff.erase(var);
  Add(replace);
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
          substitution.SetCoeffOf(entry.first, entry.second / dividend);
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
  opt_obj_.expression.ReplaceVariableWithExpression(non_base, substitution);
}

LPModel::Result LPModel::Solve() {
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
    if (e.IsUndefined()) {
      return SOLVED;
    }
    Variable d;
    Num min_ = -10000000;
    for (auto var : base_variables_) {
      for (auto constraint : constraints_) {
        if (-constraint.expression.GetCoeffOf(var) > 0) {
          if (min_ > constraint.expression.constant /
                         (-constraint.expression.variable_coeff[var])) {
            min_ = constraint.expression.constant /
                   (-constraint.expression.variable_coeff[var]);
            d = var;
          }
        }
      }
    }
    if (d.IsUndefined()) {
      return UNBOUNDED;
    }
    Pivot(e, d);
  }
  return NOSOLUTION;
}
