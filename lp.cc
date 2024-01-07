#include "lp.h"

int LPModel::base_variable_count_ = 0;
int LPModel::substitution_variable_count_ = 0;

Variable CreateBaseVariable() {
  auto var = Variable(kBase + std::to_string(LPModel::base_variable_count_));
  LPModel::base_variable_count_ += 1;
  return var;
}

Variable CreateSubstitutionVariable() {
  auto var = Variable(kSubstitution +
                      std::to_string(LPModel::substitution_variable_count_));
  LPModel::substitution_variable_count_ += 1;
  return var;
}
void LPModel::ToStandardForm() {
  // Opt obj transform.
  if (opt_obj_.type == OptimizationObject::Type::MIN) {
    opt_reverted_ = true;
    for (auto& entry : opt_obj_.expression.variable_coeff) {
      entry.second *= -1.0f;
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
    constraint.expression.constant = 0.0f;
    if (constraint.type == Constraint::Type::GE) {
      for (auto& entry : constraint.expression.variable_coeff) {
        entry.second *= -1.0f;
      }
      constraint.compare *= -1.0f;
      constraint.type = Constraint::Type::LE;
    }
  }
  std::set<Variable> positive_vars;
  std::vector<int> positive_con_index;
  for (int i = 0; i < constraints_.size(); i++) {
    auto& constraint = constraints_[i];
    if (constraint.expression.variable_coeff.size() == 1 and
        constraint.expression.constant == 0.0f) {
      auto entry = constraint.expression.variable_coeff.begin();
      if (entry->second < 0.0f) {
        if (constraint.compare <= 0.0f) {
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
    Variable s1 = CreateSubstitutionVariable(),
             s2 = CreateSubstitutionVariable();
    Expression exp(0.0f);
    exp.SetCoeffOf(var, 1.0f);
    exp.SetCoeffOf(s1, -1.0f);
    exp.SetCoeffOf(s2, 1.0f);
    for (auto& constraint : constraints_) {
      ReplaceVariableWithExpression(constraint.expression, var, exp);
    }
    non_base_variables_.insert(s1);
    non_base_variables_.insert(s2);
  }
}

void LPModel::ToRelaxedForm() {
  for (auto& constraint : constraints_) {
    auto base_var = CreateBaseVariable();
    for (auto& entry : constraint.expression.variable_coeff) {
      entry.second *= -1.0f;
    }
    constraint.expression.constant = constraint.compare;
    constraint.compare = 0.0f;
    constraint.type = Constraint::Type::EQ;
    constraint.AddItem(-1.0f, base_var);
    base_variables_.insert(base_var);
  }
}

void LPModel::Pivot(Variable base, Variable non_base) {
  assert(non_base_variables_.find(non_base) != non_base_variables_.end());
  assert(base_variables_.find(base) != base_variables_.end());
  Expression substitution(0.0f);
  for (auto& constraint : constraints_) {
    if (constraint.expression.GetCoeffOf(base) != 0.0f and
        constraint.expression.GetCoeffOf(non_base) != 0.0f) {
      Num dividend(-constraint.expression.GetCoeffOf(non_base));
      substitution.constant = (constraint.expression.constant / dividend);
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
  ReplaceVariableWithExpression(opt_obj_.expression, non_base, substitution);
  for (auto& constraint : constraints_) {
    if (constraint.expression.GetCoeffOf(non_base) == 0.0f) continue;
    if (constraint.expression.variable_coeff.find(base) ==
        constraint.expression.variable_coeff.end()) {
      ReplaceVariableWithExpression(constraint.expression, non_base,
                                    substitution);
    } else {
      Num coeff = constraint.expression.GetCoeffOf(non_base);
      constraint.expression *= (-1.0f / coeff);
    }
  }
}

LPModel::Result LPModel::Initialize() {
  // Phase 1: If there are negative constants in some constraint, use Pivot
  // method to transform the constrain system to non-negative form.
  bool need_transform = false;
  for (auto constraint : constraints_) {
    if (constraint.expression.constant < 0.0f) {
      need_transform = true;
    }
  }
  if (!need_transform) return SOLVED;
  LPModel model;
  Variable artificial_var("artificial_variable");
  for (auto constraint : constraints_) {
    constraint.AddItem(1.0f, artificial_var);
    model.AddConstraint(constraint);
  }
  model.non_base_variables_ = non_base_variables_;
  model.non_base_variables_.insert(artificial_var);
  model.base_variables_ = base_variables_;
  OptimizationObject obj;
  obj.AddItem(-1.0f, artificial_var);
  obj.SetType(OptimizationObject::MAX);
  model.SetOptimizationObject(obj);
  Num minimum = 0.0f;
  int best = -1;
  for (int i = 0; i < model.constraints_.size(); i++) {
    auto constraint = model.constraints_[i];
    if (constraint.expression.constant < minimum) {
      minimum = constraint.expression.constant;
      best = i;
    }
  }
  assert(best >= 0);
  for (auto entry : model.constraints_[best].expression.variable_coeff) {
    auto var = entry.first;
    if (base_variables_.find(var) != base_variables_.end()) {
      model.Pivot(var, artificial_var);
    }
  }
  for (auto constraint : model.constraints_) {
    assert(constraint.expression.constant >= 0.0f);
  }
  model.Solve();
  if (model.GetOptimum() < 0.0f) {
    return NOSOLUTION;
  }
  if (model.base_variables_.find(artificial_var) !=
      model.base_variables_.end()) {
    auto any_non_base_var = model.non_base_variables_.begin();
    model.Pivot(artificial_var, *any_non_base_var);
  }
  assert(model.non_base_variables_.find(artificial_var) !=
         model.non_base_variables_.end());
  for (auto& constraint : model.constraints_) {
    constraint.expression.SetCoeffOf(artificial_var, 0.0f);
  }
  constraints_ = model.constraints_;
  model.non_base_variables_.erase(artificial_var);
  base_variables_ = model.base_variables_;
  non_base_variables_ = model.non_base_variables_;
  for (auto base : model.base_variables_) {
    Expression substitution(0.0f);
    for (auto constraint : model.constraints_) {
      if (constraint.expression.GetCoeffOf(base) != 0.0f) {
        substitution = constraint.expression;
        substitution *= (-1.0f / constraint.expression.GetCoeffOf(base));
        substitution.SetCoeffOf(base, 0.0f);
        break;
      }
    }
    ReplaceVariableWithExpression(opt_obj_.expression, base, substitution);
  }
  return SOLVED;
}

LPModel::Result LPModel::Solve() {
  auto res = Initialize();
  if (res == NOSOLUTION) return NOSOLUTION;
  for (auto constraint : constraints_) {
    assert(constraint.expression.constant >= 0.0f);
  }
  // Phase 2: The main optimization procedures.
  while (true) {
    Variable e;
    for (auto& entry : opt_obj_.expression.variable_coeff) {
      if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
          entry.second > 0.0f) {
        e = entry.first;
        break;
      }
    }
    if (e.IsUndefined()) {
      return SOLVED;
    }
    Variable d;
    Num min_ = 10000000000.0f;
    for (auto base : base_variables_) {
      for (auto constraint : constraints_) {
        assert(constraint.expression.constant >= 0.0f);
        if (constraint.expression.GetCoeffOf(base) != 0.0f and
            -constraint.expression.GetCoeffOf(e) > 0.0f) {
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
  throw std::runtime_error("Should not reach this line");
}

Num LPModel::GetOptimum() {
  for (auto& entry : opt_obj_.expression.variable_coeff) {
    if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
        entry.second > 0.0f) {
      throw std::runtime_error("Not Solved");
    }
  }
  if (opt_reverted_) return -opt_obj_.expression.constant;
  return opt_obj_.expression.constant;
}

std::map<Variable, Num> LPModel::GetSolution() {
  std::map<Variable, Num> sol;
  for (auto entry : non_base_variables_) {
    if (IsUserDefined(entry)) {
      sol[entry] = 0.0f;
    }
  }
  for (auto base : base_variables_) {
    for (auto constraint : constraints_) {
      if (constraint.expression.GetCoeffOf(base) != 0.0f) {
        if (IsUserDefined(base)) {
          sol[base] = -constraint.expression.constant /
                      constraint.expression.GetCoeffOf(base);
        }
      }
    }
  }
  return sol;
}
