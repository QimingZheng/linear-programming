#include "lp.h"

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
  assert(tableau_is_base_variable_[base_column_index] == true);
  assert(tableau_is_base_variable_[non_base_column_index] == false);
  tableau_is_base_variable_[base_column_index] = false;
  tableau_is_base_variable_[non_base_column_index] = true;

  // Find a row that contains both base and non_base.
  List<real_t>* base_column =
      new List<real_t>(tableau_->Col(base_column_index));
  List<real_t>* non_base_column = tableau_->Col(non_base_column_index);
  base_column->Mul(non_base_column);
  tableau_index_t candidate_row_ind = -1;
  for (auto iter = base_column->Begin(); iter->IsEnd() == false;
       iter = iter->Next()) {
    if (iter->Index() != constant_index_ and !_IsZero(iter->Data())) {
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
  auto helper_tableau_ = candidate_col->SparseCross(candidate_row);
  tableau_->Add(helper_tableau_);

  delete helper_tableau_;
  delete base_column;
  delete substitution;
  delete candidate_col;
  delete candidate_row;
}

bool needTableauInitialization(Tableau<real_t>* tableau,
                               tableau_index_t constant_column_index) {
  for (auto row = 0; row < tableau->Rows(); row++) {
    if (_IsNegative(tableau->Row(row)->At(constant_column_index))) {
      return true;
    }
  }
  return false;
}

Result LPModel::TableauSimplexInitialize() {
  if (!needTableauInitialization(tableau_, constant_index_)) return SOLVED;

  Variable artificial_var = CreateArtificialVariable();
  non_base_variables_.insert(artificial_var);

  tableau_index_t artificial_var_index = variable_to_index_.size() + 1;
  tableau_is_base_variable_[artificial_var_index] = false;

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
    if (_IsZero(iter->Data())) continue;
    if (tableau_is_base_variable_[iter->Index()]) {
      TableauPivot(index_to_variable_[iter->Index()], artificial_var);
      break;
    }
  }
  // Now, all constraints' constant b is non-negative.
  for (auto iter = tableau_->Col(constant_index_)->Begin(); !iter->IsEnd();
       iter = iter->Next())
    assert(_IsNonNegative(iter->Data()));

  assert(TableauSimplexSolve() == SOLVED);

  if (GetTableauSimplexOptimum().IsNegative()) {
    return NOSOLUTION;
  }

  if (tableau_is_base_variable_[artificial_var_index]) {
    bool pivoted = false;
    for (auto iter = tableau_->Col(artificial_var_index)->Begin();
         !iter->IsEnd(); iter = iter->Next()) {
      if (pivoted) break;
      if (_IsZero(iter->Data())) continue;
      for (auto row_iter = tableau_->Row(iter->Index())->Begin();
           !row_iter->IsEnd(); row_iter = row_iter->Next()) {
        if (tableau_is_base_variable_[row_iter->Index()]) continue;
        if (_IsZero(row_iter->Data())) continue;
        TableauPivot(artificial_var, index_to_variable_[row_iter->Index()]);
        pivoted = true;
        break;
      }
    }
    assert(pivoted == true);
  }

  non_base_variables_.erase(artificial_var);
  variable_to_index_.erase(artificial_var);
  index_to_variable_.erase(artificial_var_index);
  tableau_is_base_variable_[artificial_var_index] = false;
  assert(non_base_variables_.find(artificial_var) == non_base_variables_.end());

  tableau_->RemoveExtraCol();
  opt_obj_tableau_ = original_opt_obj_tableau;
  // Use a List to store the added amount instead of applying to
  // `opt_obj_tableau_` directly (may corrupt the iterator).
  auto opt_obj_added_amount = new List<real_t>();
  // Replace base variables in the objective function with non-base variable.
  for (auto opt_iter = opt_obj_tableau_->Begin(); !opt_iter->IsEnd();
       opt_iter = opt_iter->Next()) {
    if (!tableau_is_base_variable_[opt_iter->Index()]) continue;
    if (_IsZero(opt_iter->Data())) continue;
    auto base_index = opt_iter->Index();
    for (auto iter = tableau_->Col(base_index)->Begin(); !iter->IsEnd();
         iter = iter->Next()) {
      if (!_IsZero(iter->Data())) {
        auto row = new List<real_t>(tableau_->Row(iter->Index()));
        row->Scale(-1.0 / iter->Data());
        row->Set(base_index, 0.0f);
        row->Scale(opt_iter->Data());
        opt_obj_added_amount->Add(row);
        break;
      }
    }
  }
  opt_obj_tableau_->Add(opt_obj_added_amount);
  delete opt_obj_added_amount;
  opt_reverted_ = original_opt_reverted;

  return SOLVED;
}

Result LPModel::TableauSimplexSolve() {
  if (needTableauInitialization(tableau_, constant_index_)) {
    auto result = TableauSimplexInitialize();
    if (result == NOSOLUTION) return NOSOLUTION;
    assert(result == SOLVED);
  }
  assert(!needTableauInitialization(tableau_, constant_index_));

  while (true) {
    Variable e;
    real_t max_ = std::numeric_limits<real_t>::min();
    // Find any non-base variable x_{e} that c_e > 0.
    for (auto iter = opt_obj_tableau_->Begin(); !iter->IsEnd();
         iter = iter->Next()) {
      if (iter->Index() == constant_index_) continue;
      if (tableau_is_base_variable_[iter->Index()]) continue;
      if (_IsPositive(iter->Data()) and iter->Data() > max_) {
        max_ = iter->Data();
        e = index_to_variable_[iter->Index()];
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
    real_t min_ = std::numeric_limits<real_t>::max();
    tableau_index_t e_col_indx = variable_to_index_[e];
    for (auto iter = tableau_->Col(e_col_indx)->Begin(); !iter->IsEnd();
         iter = iter->Next()) {
      if (_IsZero(iter->Data())) continue;
      if (_IsNonNegative(iter->Data())) continue;
      tableau_index_t row_ind = iter->Index();
      auto row = tableau_->Row(row_ind);
      auto row_constant = row->At(constant_index_);
      for (auto row_iter = row->Begin(); !row_iter->IsEnd();
           row_iter = row_iter->Next()) {
        if (row_iter->Index() != constant_index_ and
            tableau_is_base_variable_[row_iter->Index()] and
            !_IsZero(row_iter->Data())) {
          if (min_ > row_constant / (-iter->Data())) {
            min_ = row_constant / (-iter->Data());
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
      if (!_IsZero(tableau_->Row(row_id)->At(variable_to_index_[base]))) {
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