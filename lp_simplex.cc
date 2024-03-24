#include "lp.h"

// Pivot operation pivots between an non-base variable a base variable:
void LPModel::Pivot(Variable base, Variable non_base) {
  assert(non_base_variables_.find(non_base) != non_base_variables_.end());
  assert(base_variables_.find(base) != base_variables_.end());
  Expression substitution(kFloatZero);
  for (auto& constraint : model_.constraints) {
    // Find a constraint that contains both the base and non-base variable.
    if (!constraint.expression.GetCoeffOf(base).IsZero() and
        !constraint.expression.GetCoeffOf(non_base).IsZero()) {
      // TODO: add this constraint after fixed the column-generation-solve bug.
      // assert(constraint.expression.GetCoeffOf(base) == -1.0f);
      auto coeff_of_non_base = constraint.expression.GetCoeffOf(non_base);
      substitution = constraint.expression;
      substitution.SetCoeffOf(non_base, kFloatZero);
      substitution /= (-coeff_of_non_base);
      base_variables_.erase(base);
      base_variables_.insert(non_base);
      non_base_variables_.erase(non_base);
      non_base_variables_.insert(base);
      break;
    }
  }
  assert(substitution != Expression(kFloatZero));
  ReplaceVariableWithExpression(model_.opt_obj.expression, non_base,
                                substitution);
  for (auto& constraint : model_.constraints) {
    // TODO(qimingzheng): figure out why replace the following comparison with
    // .IsZero() doesn't work.
    if (constraint.expression.GetCoeffOf(non_base) == kFloatZero) continue;
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

void LPModel::TableauPivot(Variable base, Variable non_base) {
  assert(tableau_ != nullptr);
  assert(opt_obj_tableau_ != nullptr);
  assert(non_base_variables_.find(non_base) != non_base_variables_.end());
  assert(base_variables_.find(base) != base_variables_.end());
  base_variables_.erase(base);
  base_variables_.insert(non_base);
  non_base_variables_.erase(non_base);
  non_base_variables_.insert(base);

  tableau_index_t base_column_index = variable_to_index_[base],
                  non_base_column_index = variable_to_index_[non_base];

  // Find a row that contains both base and non_base.
  List<real_t>* base_column =
      new List<real_t>(tableau_->Col(base_column_index));
  List<real_t>* non_base_column = tableau_->Col(non_base_column_index);
  base_column->Mul(non_base_column);
  tableau_index_t candidate_row_ind = -1;
  for (auto iter = base_column->Begin(); iter->IsEnd() == false;
       iter = iter->Next()) {
    if (iter->Index() != constant_index_ and !Num(iter->Data()).IsZero()) {
      candidate_row_ind = iter->Index();
      break;
    }
  }
  assert(candidate_row_ind >= 0);
  tableau_index_t candidate_col_ind = non_base_column_index;

  auto substitution = new List<real_t>(tableau_->Row(candidate_row_ind));
  substitution->Scale(opt_obj_tableau_->At(candidate_col_ind) *
                      (-1.0 / substitution->At(candidate_col_ind)));
  opt_obj_tableau_->Add(substitution);
  opt_obj_tableau_->Set(candidate_col_ind, 0);

  auto candidate_row = new List<real_t>(tableau_->Row(candidate_row_ind));
  candidate_row->Scale(-1.0 / candidate_row->At(candidate_col_ind));
  auto candidate_col = new List<real_t>(tableau_->Col(candidate_col_ind));
  candidate_col->Set(
      candidate_row_ind,
      1.0f + tableau_->Row(candidate_row_ind)->At(candidate_col_ind));
  auto helper_tableau_ = candidate_col->Cross(
      candidate_row, model_.constraints.size(), variable_to_index_.size() + 1);
  tableau_->Add(helper_tableau_);

  delete helper_tableau_;
  delete base_column;
  delete substitution;
  delete candidate_col;
  delete candidate_row;
}

bool needInitialization(const std::vector<Constraint>& constraints) {
  bool needed = false;
  for (auto constraint : constraints) {
    if (constraint.expression.constant.IsNegative()) {
      needed = true;
      break;
    }
  }
  return needed;
}

bool needTableauInitialization(Tableau<real_t>* tableau,
                               tableau_index_t constant_column_index) {
  for (auto row = 0; row < tableau->Rows(); row++) {
    if (Num(tableau->Row(row)->At(constant_column_index)).IsNegative()) {
      return true;
    }
  }
  return false;
}

/* Phase 1: If there are negative constants in some constraint, use Pivot method
 * to transform the constrain system to non-negative form. See:
 * https://zh.wikipedia.org/wiki/%E5%8D%95%E7%BA%AF%E5%BD%A2%E6%B3%95#%E5%88%9D%E5%A7%8B%E5%8C%96%E8%BF%87%E7%A8%8B
 */
Result LPModel::Initialize() {
  if (!needInitialization(model_.constraints)) return SOLVED;

  // Define a new non-base variable (x_{0}) and construct a new helper LP
  // problem:
  //  max -x_{0}
  //  s.t.
  //    x_{i} >=0 \forall 0 <= i <= n + m
  //    x_{j} = b_{j} - \sum A_{j, k} x_{k} (x_{k} is non-basis) + x_{0} \forall
  //    j \in basis
  LPModel helper_lp;
  Variable artificial_var = CreateArtificialVariable();
  for (auto constraint : model_.constraints) {
    constraint.expression += artificial_var;
    helper_lp.AddConstraint(constraint);
  }
  helper_lp.non_base_variables_ = non_base_variables_;
  helper_lp.non_base_variables_.insert(artificial_var);
  helper_lp.base_variables_ = base_variables_;
  OptimizationObject obj(FLOAT);
  obj.expression += -1.0f * artificial_var;
  obj.SetOptType(OptimizationObject::MAX);
  helper_lp.SetOptimizationObject(obj);

  // Find a x_{d} that minimize b_{d}
  Num minimum = kFloatZero;
  int minimum_base_index = -1;
  for (size_t i = 0; i < helper_lp.model_.constraints.size(); i++) {
    auto constraint = helper_lp.model_.constraints[i];
    if (constraint.expression.constant < minimum) {
      minimum = constraint.expression.constant;
      minimum_base_index = i;
    }
  }
  assert(minimum_base_index >= 0);

  // Perform Pivot(x_{d}, x_{0})
  for (auto entry : helper_lp.model_.constraints[minimum_base_index]
                        .expression.variable_coeff) {
    auto var = entry.first;
    if (base_variables_.find(var) != base_variables_.end()) {
      helper_lp.Pivot(var, artificial_var);
      break;
    }
  }
  // Now, all constraints' constant b is non-negative.
  for (auto constraint : helper_lp.model_.constraints)
    assert(constraint.expression.constant.IsNonNegative());

  // Solve the helper LP problem (can directly goes into phase 2, and the
  // problem must be solvable and bounded).
  auto result = helper_lp.SimplexSolve();
  assert(result == SOLVED);

  // If the helper LP problem's solution is nagative, which the raw LP
  // constraints is not satisfiable unless x_{0} is positive .
  if (helper_lp.GetSimplexOptimum().IsNegative()) {
    return NOSOLUTION;
  }

  // Transform the helper LP problem back to the raw LP problem.

  // If x_{0} is base variable, perform pivot(0, x_{e}) where x_{e} is any
  // non-base variable. This operation does not breaks the no-negative property
  // of vector b.
  if (helper_lp.base_variables_.find(artificial_var) !=
      helper_lp.base_variables_.end()) {
    auto any_non_base_var = helper_lp.non_base_variables_.begin();
    helper_lp.Pivot(artificial_var, *any_non_base_var);
  }
  assert(helper_lp.non_base_variables_.find(artificial_var) !=
         helper_lp.non_base_variables_.end());
  for (auto& constraint : helper_lp.model_.constraints) {
    constraint.expression.SetCoeffOf(artificial_var, kFloatZero);
  }
  // Copy the helper_lp's constraints.
  model_.constraints = helper_lp.model_.constraints;
  helper_lp.non_base_variables_.erase(artificial_var);
  base_variables_ = helper_lp.base_variables_;
  non_base_variables_ = helper_lp.non_base_variables_;
  // Replace base variables in the objective function with non-base variable.
  for (auto base : helper_lp.base_variables_) {
    Expression substitution(kFloatZero);
    for (auto constraint : helper_lp.model_.constraints) {
      if (!constraint.expression.GetCoeffOf(base).IsZero()) {
        substitution = constraint.expression;
        substitution *= (-kFloatOne / constraint.expression.GetCoeffOf(base));
        substitution.SetCoeffOf(base, kFloatZero);
        break;
      }
    }
    ReplaceVariableWithExpression(model_.opt_obj.expression, base,
                                  substitution);
  }

  return SOLVED;
}

Result LPModel::TableauSimplexInitialize() {
  if (!needTableauInitialization(tableau_, constant_index_)) return SOLVED;

  Variable artificial_var = CreateArtificialVariable();
  non_base_variables_.insert(artificial_var);

  tableau_index_t artificial_var_index = variable_to_index_.size() + 1;
  variable_to_index_[artificial_var] = artificial_var_index;
  index_to_variable_[artificial_var_index] = artificial_var;

  auto original_opt_obj_tableau = opt_obj_tableau_;
  opt_obj_tableau_ = new List<real_t>();

  opt_obj_tableau_->Append(artificial_var_index, -1.0);
  auto original_opt_reverted = opt_reverted_;
  opt_reverted_ = false;
  auto extra_col = new List<real_t>();
  for (int i = 0; i < tableau_->Rows(); i++) {
    extra_col->Append(i, 1.0);
  }
  tableau_->AppendExtraCol(extra_col);

  List<real_t>::ReduceStruct min_bounding_constant = {-1, 0.0f};
  min_bounding_constant =
      tableau_->Col(constant_index_)
          ->Reduce(List<real_t>::MinReduce, min_bounding_constant);

  assert(min_bounding_constant.first >= 0);
  auto min_bounding_constant_constaint_index = min_bounding_constant.first;
  for (auto iter =
           tableau_->Row(min_bounding_constant_constaint_index)->Begin();
       !iter->IsEnd(); iter = iter->Next()) {
    if (iter->Index() == constant_index_) continue;
    if (Num(iter->Data()).IsZero()) continue;
    if (IsBaseVariable(index_to_variable_[iter->Index()])) {
      TableauPivot(index_to_variable_[iter->Index()], artificial_var);
      break;
    }
  }
  // Now, all constraints' constant b is non-negative.
  for (auto iter = tableau_->Col(constant_index_)->Begin(); !iter->IsEnd();
       iter = iter->Next())
    assert(Num(iter->Data()).IsNonNegative());

  assert(TableauSimplexSolve() == SOLVED);

  if (GetTableauSimplexOptimum().IsNegative()) {
    return NOSOLUTION;
  }

  if (base_variables_.find(artificial_var) != base_variables_.end()) {
    auto any_non_base_var = non_base_variables_.begin();
    TableauPivot(artificial_var, *any_non_base_var);
  }

  non_base_variables_.erase(artificial_var);
  variable_to_index_.erase(artificial_var);
  index_to_variable_.erase(artificial_var_index);
  assert(non_base_variables_.find(artificial_var) == non_base_variables_.end());

  tableau_->RemoveExtraCol();
  // Replace base variables in the objective function with non-base variable.
  opt_obj_tableau_ = original_opt_obj_tableau;
  for (auto base : base_variables_) {
    auto base_index = variable_to_index_[base];
    if (!Num(opt_obj_tableau_->At(base_index)).IsZero()) {
      for (auto iter = tableau_->Col(base_index)->Begin(); !iter->IsEnd();
           iter = iter->Next()) {
        if (!Num(iter->Data()).IsZero()) {
          auto row = new List<real_t>(tableau_->Row(iter->Index()));
          row->Scale(-1.0 / iter->Data());
          row->Set(base_index, 0.0f);
          row->Scale(opt_obj_tableau_->At(base_index));
          opt_obj_tableau_->Add(row);
          break;
        }
      }
    }
  }
  opt_reverted_ = original_opt_reverted;

  return SOLVED;
}

Result LPModel::SimplexSolve() {
  {
    Timer timer;
    auto res = Initialize();
    timer.Stop();
    if (enable_logging_)
      std::cout << "[Initialization]: " << std::to_string(timer.Delta() / 1000)
                << " ms\n";
    if (res == NOSOLUTION) return NOSOLUTION;
    assert(res == SOLVED);
  }
  assert(needInitialization(model_.constraints) == false);

  int iter = 0;
  Timer timer;
  // Phase 2: The main optimization procedures.
  while (true) {
    iter += 1;
    Variable e;
    // Find any non-base variable x_{e} that c_e > 0.
    for (auto& entry : model_.opt_obj.expression.variable_coeff) {
      if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
          entry.second.IsPositive()) {
        e = entry.first;
        break;
      }
    }
    // If not found, which means \vec c <= \vec 0, so the maximum of the
    // objective function is already achieved.
    if (e.IsUndefined()) {
      simplex_optimum_ = GetOptimum();
      simplex_solution_ = GetSolution();
      timer.Stop();
      if (enable_logging_)
        LogIterStatus(iter, timer.Delta(), simplex_optimum_.float_value);
      return SOLVED;
    }
    // Find a base variable x_{d} s.t. A_{d,e} > 0 and minimize b_{d}/A_{d,e}
    Variable d;
    Num min_ = kFloatMax;
    for (auto base : base_variables_) {
      for (auto constraint : model_.constraints) {
        assert(constraint.expression.constant.IsNonNegative());
        if (!constraint.expression.GetCoeffOf(base).IsZero() and
            constraint.expression.GetCoeffOf(e).IsNegative()) {
          if (min_ > constraint.expression.constant /
                         (-constraint.expression.GetCoeffOf(e))) {
            min_ = constraint.expression.constant /
                   (-constraint.expression.GetCoeffOf(e));
            d = base;
          }
        }
      }
    }
    // If x_{d} is not found, which means the optimum is unbounded (by assigning
    // x_{e} as +infinity, and all other non-base as 0).
    if (d.IsUndefined()) {
      simplex_extreme_ray_ = GetRay(e);
      return UNBOUNDED;
    }
    // Perform pivot(x_{d}, x_{e})
    Pivot(d, e);
    if (enable_logging_ && iter % log_every_iters_ == 0) {
      timer.Stop();
      LogIterStatus(iter, timer.Delta(), GetOptimum(false).float_value);
      timer.Reset();
    }
  }
  return ERROR;
}

Result LPModel::TableauSimplexSolve() {
  // TODO: run phase 1 initialization before solving the LP model.
  if (needTableauInitialization(tableau_, constant_index_)) {
    auto result = TableauSimplexInitialize();
    if (result == NOSOLUTION) return NOSOLUTION;
    assert(result == SOLVED);
  }

  while (true) {
    Variable e;
    // Find any non-base variable x_{e} that c_e > 0.
    for (auto iter = opt_obj_tableau_->Begin(); !iter->IsEnd();
         iter = iter->Next()) {
      if (iter->Index() != constant_index_ and
          Num(iter->Data()).IsPositive() and
          non_base_variables_.find(index_to_variable_[iter->Index()]) !=
              non_base_variables_.end()) {
        e = index_to_variable_[iter->Index()];
        break;
      }
    }
    // If not found, which means \vec c <= \vec 0, so the maximum of the
    // objective function is already achieved.
    if (e.IsUndefined()) {
      simplex_solution_ = GetTableauSimplexSolution();
      simplex_optimum_ = GetTableauSimplexOptimum();
      return SOLVED;
    }
    // Find a base variable x_{d} s.t. A_{d,e} > 0 and minimize b_{d}/A_{d,e}
    Variable d;
    Num min_ = kFloatMax;
    tableau_index_t e_col_indx = variable_to_index_[e];
    for (auto iter = tableau_->Col(e_col_indx)->Begin(); !iter->IsEnd();
         iter = iter->Next()) {
      if (Num(iter->Data()).IsZero()) continue;
      if (!Num(iter->Data()).IsNegative()) continue;
      tableau_index_t row_ind = iter->Index();
      auto row = tableau_->Row(row_ind);
      for (auto row_iter = row->Begin(); !row_iter->IsEnd();
           row_iter = row_iter->Next()) {
        if (row_iter->Index() != constant_index_ and
            IsBaseVariable(index_to_variable_[row_iter->Index()]) and
            !Num(row_iter->Data()).IsZero()) {
          if (min_ > row->At(constant_index_) / (-iter->Data())) {
            min_ = row->At(constant_index_) / (-iter->Data());
            d = index_to_variable_[row_iter->Index()];
          }
        }
      }
    }
    // If x_{d} is not found, which means the optimum is unbounded (by assigning
    // x_{e} as +infinity, and all other non-base as 0).
    if (d.IsUndefined()) {
      // TODO
      return UNBOUNDED;
    }
    // Perform pivot(x_{d}, x_{e})
    TableauPivot(d, e);
  }
  return ERROR;
}

std::map<Variable, Num> LPModel::GetTableauSimplexSolution() {
  std::map<Variable, Num> all_sol;
  std::map<Variable, Num> sol;
  for (auto var : non_base_variables_) {
    all_sol[var] = 0.0f;
    if (IsUserDefined(var) or IsOverriddenAsUserDefined(var)) sol[var] = 0.0f;
  }
  for (auto base : base_variables_) {
    int row_id = 0;
    for (auto constraint : model_.constraints) {
      if (!Num(tableau_->Row(row_id)->At(variable_to_index_[base])).IsZero()) {
        all_sol[base] = -tableau_->Row(row_id)->At(constant_index_) /
                        tableau_->Row(row_id)->At(variable_to_index_[base]);
        if (IsUserDefined(base) or IsOverriddenAsUserDefined(base))
          sol[base] = all_sol[base];
      }
      row_id++;
    }
  }
  for (auto entry : raw_variable_expression_) {
    auto raw_var = entry.first;
    auto exp = entry.second;
    while (exp.variable_coeff.size() > 0) {
      auto entry = *exp.variable_coeff.begin();
      ReplaceVariableWithExpression(exp, entry.first, all_sol[entry.first]);
    }
    sol[raw_var] = exp.constant;
  }
  return sol;
}

Num LPModel::GetTableauSimplexOptimum() {
  return (opt_reverted_ ? -1 : 1) * opt_obj_tableau_->At(constant_index_);
}

Num LPModel::GetSimplexOptimum() { return simplex_optimum_; }

std::map<Variable, Num> LPModel::GetSimplexSolution() {
  return simplex_solution_;
}

std::map<Variable, Num> LPModel::GetSimplexExtremeRay() {
  return simplex_extreme_ray_;
}

std::map<Variable, Num> LPModel::GetRay(Variable non_basis_var) {
  std::map<Variable, Num> all_sol;
  std::map<Variable, Num> sol;
  for (auto var : non_base_variables_) {
    all_sol[var] = (var == non_basis_var) ? 1.0f : 0.0f;
    if (IsUserDefined(var) or IsOverriddenAsUserDefined(var))
      sol[var] = (var == non_basis_var) ? 1.0f : 0.0f;
  }
  for (auto base : base_variables_) {
    for (auto constraint : model_.constraints) {
      if (constraint.expression.GetCoeffOf(base) != 0.0f) {
        all_sol[base] = -constraint.expression.GetCoeffOf(non_basis_var) /
                        constraint.expression.GetCoeffOf(base);
        if (IsUserDefined(base) or IsOverriddenAsUserDefined(base))
          sol[base] = -constraint.expression.GetCoeffOf(non_basis_var) /
                      constraint.expression.GetCoeffOf(base);
      }
    }
  }
  for (auto entry : raw_variable_expression_) {
    auto raw_var = entry.first;
    auto exp = entry.second;
    while (exp.variable_coeff.size() > 0) {
      auto entry = *exp.variable_coeff.begin();
      ReplaceVariableWithExpression(exp, entry.first, all_sol[entry.first]);
    }
    sol[raw_var] = exp.constant;
  }
  return sol;
}
