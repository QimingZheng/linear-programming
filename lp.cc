#include "lp.h"

int Variable::base_variable_count_ = 0;
int Variable::substitution_variable_count_ = 0;

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
  std::vector<Constraint> equation_constraint;
  for (auto& constraint : constraints_) {
    if (constraint.type == Constraint::Type::EQ) {
      constraint.type = Constraint::Type::GE;
      Constraint con;
      con.SetCompare(constraint.compare);
      con.SetType(Constraint::Type::LE);
      con.expression = constraint.expression;
      equation_constraint.push_back(con);
    }
  }
  for (auto constrain : equation_constraint) {
    constraints_.push_back(constrain);
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
    }
  }
  std::set<Variable> positive_vars;
  std::vector<int> positive_con_index;
  for (int i = 0; i < constraints_.size(); i++) {
    auto& constraint = constraints_[i];
    if (constraint.expression.variable_coeff.size() == 1 and
        constraint.expression.constant == 0.0) {
      auto entry = constraint.expression.variable_coeff.begin();
      if (entry->second < 0.0) {
        if (constraint.compare <= 0.0) {
          positive_vars.insert(entry->first);
          positive_con_index.push_back(i);
        }
      }
      /*
      else {
        if (constraint.compare >= 0.0) {
          positive_vars.insert(entry->first);
          positive_con_index.push_back(i);
        }
      }
      */
    }
  }
  std::reverse(positive_con_index.begin(), positive_con_index.end());
  for (auto ind : positive_con_index) {
    constraints_.erase(constraints_.begin() + ind);
  }
  for (auto constraint : constraints_) {
    for (auto entry : constraint.expression.variable_coeff) {
      non_base_variables_.insert(entry.first);
    }
  }
  std::set<Variable> no_limit_vars;
  for (auto non_base : non_base_variables_) {
    if (positive_vars.find(non_base) == positive_vars.end()) {
      // non_base has no constraints on its positive/negative property.
      no_limit_vars.insert(non_base);
    }
  }
  for (auto var : no_limit_vars) {
    non_base_variables_.erase(var);
    Variable s1 = Variable::CreateSubstitutionVariable(),
             s2 = Variable::CreateSubstitutionVariable();
    Expression exp;
    exp.SetCoeffOf(var, 1.0);
    exp.SetCoeffOf(s1, -1.0);
    exp.SetCoeffOf(s2, 1.0);
    for (auto& constraint : constraints_) {
      constraint.expression.ReplaceVariableWithExpression(var, exp);
    }
    non_base_variables_.insert(s1);
    non_base_variables_.insert(s2);
  }
}

void LPModel::ToRelaxedForm() {
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

LPModel::Result LPModel::Initialize() {
  // Phase 1: If there are negative constants in some constraint, use Pivot
  // method to transform the constrain system to non-negative form.
  bool need_transform = false;
  for (auto constraint : constraints_) {
    if (constraint.compare < 0.0) {
      need_transform = true;
    }
  }
  if (!need_transform) return SOLVED;
  LPModel model;
  Variable artificial_var("artificial_variable");
  for (auto constraint : constraints_) {
    constraint.AddItem(1.0, artificial_var);
    model.AddConstraint(constraint);
  }
  model.non_base_variables_ = non_base_variables_;
  for (auto base : base_variables_) model.non_base_variables_.insert(base);
  model.base_variables_.insert(artificial_var);
  OptimizationObject obj;
  obj.AddItem(-1.0, artificial_var);
  obj.SetType(OptimizationObject::MAX);
  model.SetOptimizationObject(obj);
  Num minimum = 0.0;
  int best = 0;
  for (int i = 0; i < constraints_.size(); i++) {
    auto constraint = constraints_[i];
    if (constraint.expression.constant <= 0.0) {
      minimum = std::min(minimum, constraint.expression.constant);
      best = i;
    }
  }
  for (auto entry : constraints_[best].expression.variable_coeff) {
    auto var = entry.first;
    if (base_variables_.find(var) != base_variables_.end()) {
      model.Pivot(var, artificial_var);
    }
  }
  model.Solve();
  if (model.GetOptimum() < 0.0) {
    return NOSOLUTION;
  }
  if (model.base_variables_.find(artificial_var) !=
      model.base_variables_.end()) {
    auto any_non_base_var = model.non_base_variables_.begin();
    model.Pivot(*any_non_base_var, artificial_var);
  }
  assert(model.non_base_variables_.find(artificial_var) !=
         model.non_base_variables_.end());
  for (auto& constraint : model.constraints_) {
    constraint.expression.SetCoeffOf(artificial_var, 0.0);
  }
  constraints_ = model.constraints_;
  return SOLVED;
}

LPModel::Result LPModel::Solve() {
  auto res = Initialize();
  if (res == NOSOLUTION) return NOSOLUTION;
  for (auto constraint : constraints_) {
    assert(constraint.compare >= 0.0);
  }
  // Phase 2: The main optimization procedures.
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

std::map<Variable, Num> LPModel::GetSolution() {
  std::map<Variable, Num> sol;
  for (auto entry : non_base_variables_) {
    if (entry.IsUserDefined()) {
      sol[entry] = 0.0;
    }
  }
  for (auto base : base_variables_) {
    for (auto constraint : constraints_) {
      if (constraint.expression.GetCoeffOf(base) != 0.0) {
        sol[base] = -constraint.expression.constant /
                    constraint.expression.GetCoeffOf(base);
      }
    }
  }
  return sol;
}
