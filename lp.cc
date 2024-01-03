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

void LPModel::ToStandardForm() {
  // Opt obj transform.
  if (opt_obj_.type == OptimizationObject::Type::MIN) {
    opt_reverted_ = true;
    for (auto& entry : opt_obj_.expression.variable_coeff) {
      entry.second *= -1.0;
    }
    opt_obj_.type = OptimizationObject::Type::MAX;
  }
  for (auto& constraint : constraints_) {
    constraint.compare -= constraint.expression.constant;
    constraint.expression.constant = 0.0;
    if (constraint.type == Constraint::Type::GE) {
      for (auto& entry : constraint.expression.variable_coeff) {
        entry.second *= -1.0;
      }
      constraint.compare *= -1.0;
      constraint.type = Constraint::Type::LE;
    } else if (constraint.type == Constraint::Type::EQ) {
      // TODO: split this constraint into two constraints.
    }
  }
  // TODO: add constraints for un-constrained variables.
}

void LPModel::ToRelaxedForm() {
  for (auto constraint : constraints_) {
    for (auto entry : constraint.expression.variable_coeff) {
      non_base_variables_.insert(entry.first);
    }
  }
  for (auto& constraint : constraints_) {
    auto base_var = Variable::CreateBaseVariable();
    for (auto& entry : constraint.expression.variable_coeff) {
      entry.second *= -1.0;
    }
    constraint.expression.constant = constraint.compare;
    constraint.compare = 0.0;
    constraint.type = Constraint::Type::EQ;
    constraint.AddItem(-1.0, base_var);
    base_variables_.insert(base_var);
  }
}

void LPModel::Pivot(Variable base, Variable non_base) {
  assert(non_base_variables_.find(non_base) != non_base_variables_.end());
  assert(base_variables_.find(base) != base_variables_.end());
  Expression substitution;
  for (auto& constraint : constraints_) {
    if (constraint.expression.GetCoeffOf(base) != 0.0 and
        constraint.expression.GetCoeffOf(non_base) != 0.0) {
      Num dividend = (-constraint.expression.GetCoeffOf(non_base));
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
  opt_obj_.expression.ReplaceVariableWithExpression(non_base, substitution);
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
    Num min_ = 10000000000;
    for (auto base : base_variables_) {
      for (auto constraint : constraints_) {
        assert(constraint.expression.constant >= 0.0);
        if (constraint.expression.GetCoeffOf(base) != 0 and
            -constraint.expression.GetCoeffOf(e) > 0) {
          if (min_ > constraint.expression.constant /
                         (-constraint.expression.GetCoeffOf(e))) {
            min_ = constraint.expression.constant /
                   (-constraint.expression.GetCoeffOf(e));
            d = base;
          }
        }
      }
    }
    if (d.IsUndefined()) {
      return UNBOUNDED;
    }
    Pivot(d, e);
  }
  return NOSOLUTION;
}

Num LPModel::GetOptimum() {
  for (auto& entry : opt_obj_.expression.variable_coeff) {
    if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
        entry.second > 0) {
      throw std::runtime_error("Not Solved");
    }
  }
  if (opt_reverted_) return -opt_obj_.expression.constant;
  return opt_obj_.expression.constant;
}
