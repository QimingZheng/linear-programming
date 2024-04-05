#include "lp.h"

template <>
inline bool _IsZeroT(const real_t& x) {
  return std::abs(x) < kEpsilonF;
}

bool needTableauRevisedInitialization(Tableau<real_t>* tableau,
                                      tableau_index_t constant_column_index) {
  for (auto iter = tableau->Col(constant_column_index)->Begin(); !iter->IsEnd();
       iter = iter->Next()) {
    if (_IsNegative(iter->Data())) return true;
  }
  return false;
}

Tableau<real_t>* BasisInverse(Tableau<real_t>* tableau,
                              tableau_index_t* basis_indices,
                              tableau_size_t basis_number) {
  assert(tableau->StorageFormat() == COLUMN_ONLY);
  Eigen::MatrixXd basis_mat = Eigen::MatrixXd::Zero(basis_number, basis_number);
  for (auto i = 0; i < basis_number; i++) {
    for (auto iter = tableau->Col(basis_indices[i])->Begin(); !iter->IsEnd();
         iter = iter->Next()) {
      basis_mat(iter->Index(), i) = -iter->Data();
    }
  }
  auto inverse = basis_mat.inverse();
  Tableau<real_t>* ret =
      new Tableau<real_t>(basis_number, basis_number, ROW_ONLY);
  for (auto i = 0; i < basis_number; i++) {
    List<real_t>* row = new List<real_t>();
    for (auto j = 0; j < basis_number; j++) {
      if (_IsZero(inverse(i, j))) continue;
      row->Append(j, inverse(i, j));
    }
    ret->AppendRow(i, row);
  }
  return ret;
}

void LPModel::TableauRevisedSimplexPivot(tableau_index_t leaving_basis,
                                         tableau_index_t entering_basis,
                                         List<real_t>* mu, real_t min_ratio) {
  tableau_size_t basis_number = base_variables_.size();
  assert(leaving_basis < basis_number);
  assert(entering_basis < tableau_->Cols());
  assert(entering_basis != constant_index_);

  tableau_is_base_variable_[entering_basis] = true;
  tableau_is_base_variable_[basis_indices[leaving_basis]] = false;
  basic_feasible_solution->AddScaled(mu, min_ratio, true);
  basic_feasible_solution->Set(leaving_basis, min_ratio);
  basis_indices[leaving_basis] = entering_basis;
  basis_coeff->Set(leaving_basis, opt_obj_tableau_->At(entering_basis));

  basis_inverse->Row(leaving_basis)->Scale(-1.0 / mu->At(leaving_basis));
  mu->Set(leaving_basis, 0);
  auto helper_tableau =
      mu->SparseCross(basis_inverse->Row(leaving_basis), ROW_ONLY);
  basis_inverse->Add(helper_tableau);
}

void LPModel::TableauRevisedSimplexRemoveRedundantConstraint(
    tableau_index_t leaving_basis) {
  tableau_size_t basis_number = base_variables_.size();
  assert(leaving_basis < basis_number);

  tableau_is_base_variable_[basis_indices[leaving_basis]] = false;

  base_variables_.erase(index_to_variable_[basis_indices[leaving_basis]]);
  auto new_basic_feasible_solution = new List<real_t>(basis_number - 1, DENSE);
  auto new_basis_coeff = new List<real_t>(basis_number - 1, DENSE);
  auto new_basis_indices = new tableau_index_t[basis_number - 1];
  tableau_index_t next_id = 0;
  for (auto i = 0; i < basis_number; i++) {
    if (basis_indices[i] == leaving_basis) continue;
    new_basis_indices[next_id] = basis_indices[i];
    new_basis_coeff->Set(next_id, basis_coeff->At(i));
    new_basic_feasible_solution->Set(next_id, basic_feasible_solution->At(i));
    next_id++;
  }
  basis_indices = new_basis_indices;
  basis_coeff = new_basis_coeff;
  basic_feasible_solution = new_basic_feasible_solution;
  basis_inverse = BasisInverse(tableau_, basis_indices, basis_number - 1);
  for (auto col = 0; col < tableau_->Cols(); col++) {
    tableau_->Col(col)->Erase(leaving_basis);
  }
}

Result LPModel::TableauRevisedSimplexInitialize() {
  if (!needTableauRevisedInitialization(tableau_, constant_index_))
    return SOLVED;
  std::vector<tableau_index_t> negative_bound_ind;
  for (auto iter = tableau_->Col(constant_index_)->Begin(); !iter->IsEnd();
       iter = iter->Next()) {
    if (_IsNegative(iter->Data())) {
      negative_bound_ind.push_back(iter->Index());
    }
  }
  for (auto col = 0; col < tableau_->Cols(); col++) {
    tableau_index_t i = 0;
    for (auto iter = tableau_->Col(col)->Begin(); !iter->IsEnd();
         iter = iter->Next()) {
      while (i < negative_bound_ind.size() and
             negative_bound_ind[i] < iter->Index())
        i++;
      if (i >= negative_bound_ind.size()) break;
      if (iter->Index() < negative_bound_ind[i]) continue;
      if (iter->Index() == negative_bound_ind[i])
        tableau_->Col(col)->Set(iter->Index(), -1 * iter->Data());
    }
  }
  assert(!needTableauRevisedInitialization(tableau_, constant_index_));

  auto raw_opt = opt_obj_tableau_;
  auto raw_opt_reverted = opt_reverted_;
  opt_obj_tableau_ = new List<real_t>();
  non_base_variables_.insert(base_variables_.begin(), base_variables_.end());
  base_variables_.clear();
  if (tableau_is_base_variable_ != nullptr) delete tableau_is_base_variable_;
  tableau_is_base_variable_ =
      new bool[non_base_variables_.size() + 1 + tableau_->Rows()];
  std::memset(tableau_is_base_variable_, 0,
              sizeof(bool) * non_base_variables_.size() + 1 + tableau_->Rows());
  for (auto i = 0; i < tableau_->Rows(); i++) {
    Variable artificial = CreateArtificialVariable();
    opt_obj_tableau_->Append(tableau_->Cols(), -1);
    variable_to_index_[artificial] = tableau_->Cols();
    index_to_variable_[tableau_->Cols()] = artificial;
    tableau_is_base_variable_[tableau_->Cols()] = true;
    base_variables_.insert(artificial);
    List<real_t>* col = new List<real_t>();
    col->Append(i, -1);
    tableau_->AppendExtraCol(col);
  }

  auto result = TableauRevisedSimplexSolve();
  assert(result == SOLVED);

  if (!GetTableauRevisedSimplexOptimum().IsZero()) return NOSOLUTION;

  tableau_size_t basis_number = base_variables_.size();
  for (auto i = 0; i < basis_number; i++) {
    if (basis_indices[i] >= non_base_variables_.size()) {
      // It is an artificial var.
      bool pivoted = false;
      for (auto col = 0; col < tableau_->Cols(); col++) {
        if (col == constant_index_) continue;
        if (tableau_is_base_variable_[col]) continue;
        if (base_variables_.find(index_to_variable_[col]) !=
            base_variables_.end())
          continue;
        auto probing = basis_inverse->Times(tableau_->Col(col));
        if (!_IsZero(probing->At(i))) {
          pivoted = true;
          TableauRevisedSimplexPivot(
              i, col, probing,
              -(basic_feasible_solution->At(i) / probing->At(i)));
        }
        delete probing;
        if (pivoted) break;
      }
      assert(pivoted);
      // TODO: find a test case that can trigger the following operation (when
      // not pivoted, remove redundant constraint).
      /* if (!pivoted) TableauRevisedSimplexRemoveRedundantConstraint(i); */
    }
  }
  for (auto i = 0; i < basis_number; i++) tableau_->RemoveExtraCol();

  non_base_variables_.clear();
  base_variables_.clear();
  for (auto i = 0; i < tableau_->Cols(); i++) {
    if (i == constant_index_) continue;
    if (tableau_is_base_variable_[i]) {
      base_variables_.insert(index_to_variable_[i]);
    } else {
      non_base_variables_.insert(index_to_variable_[i]);
    }
  }
  std::vector<tableau_index_t> to_be_cleared;
  for (auto entry : index_to_variable_) {
    if (entry.first >= tableau_->Cols()) to_be_cleared.push_back(entry.first);
  }
  for (auto ind : to_be_cleared) {
    variable_to_index_.erase(index_to_variable_[ind]);
    index_to_variable_.erase(ind);
  }

  opt_obj_tableau_ = raw_opt;
  opt_reverted_ = raw_opt_reverted;
  for (auto i = 0; i < base_variables_.size(); i++)
    basis_coeff->Set(i, opt_obj_tableau_->At(basis_indices[i]));
  return SOLVED;
}

Result LPModel::TableauRevisedSimplexSolve() {
  assert(model_.opt_obj.opt_type == OptimizationObject::MAX);
  opt_obj_tableau_->Scale(-1.0);
  opt_reverted_ = !opt_reverted_;

  tableau_size_t basis_number = base_variables_.size();

  if (needTableauRevisedInitialization(tableau_, constant_index_)) {
    auto result = TableauRevisedSimplexInitialize();
    if (result == NOSOLUTION) return NOSOLUTION;
    assert(result == SOLVED);
  } else {
    assert(base_variables_.size() == tableau_->Rows());
    basic_feasible_solution = new List<real_t>(basis_number, DENSE);
    basis_coeff = new List<real_t>(basis_number, DENSE);
    basis_indices = new tableau_index_t[basis_number];
    int i = 0;
    for (auto var : base_variables_) {
      basis_indices[i] = variable_to_index_[var];
      basis_coeff->Set(i, opt_obj_tableau_->At(basis_indices[i]));
      bool initialized = false;
      for (auto iter = tableau_->Col(basis_indices[i])->Begin(); !iter->IsEnd();
           iter = iter->Next()) {
        if (!_IsZero(iter->Data())) {
          basic_feasible_solution->Set(
              i, tableau_->Col(constant_index_)->At(iter->Index()) /
                     (-iter->Data()));
          initialized = true;
          break;
        }
      }
      assert(initialized);
      i++;
    }
    basis_inverse = BasisInverse(tableau_, basis_indices, basis_number);
  }
  while (true) {
    List<real_t>* p = basis_inverse->SumScaledRows(basis_coeff);
    List<real_t>* cost = new List<real_t>(opt_obj_tableau_);
    cost->Add(tableau_->SumScaledRows(p));

    tableau_index_t entering_non_basis = -1;
    real_t min_cost = std::numeric_limits<real_t>::max();
    for (auto i = 0; i < cost->Size(); i++) {
      if (i == constant_index_) continue;
      if (tableau_is_base_variable_[i]) continue;
      real_t cost_ = cost->At(i);
      if (_IsNonNegative(cost_)) continue;
      if (min_cost > cost_) {
        min_cost = cost_;
        entering_non_basis = i;
      }
    }
    if (entering_non_basis < 0) {
      revised_simplex_optimum_ = opt_obj_tableau_->At(constant_index_);
      for (auto i = 0; i < basis_number; i++)
        revised_simplex_optimum_ += opt_obj_tableau_->At(basis_indices[i]) *
                                    basic_feasible_solution->At(i);
      if (opt_reverted_) revised_simplex_optimum_ *= -1;

      std::map<Variable, Num> all_sol;
      for (auto entry : variable_to_index_) all_sol[entry.first] = 0.0f;
      for (auto i = 0; i < basis_number; i++)
        all_sol[index_to_variable_[basis_indices[i]]] =
            basic_feasible_solution->At(i);
      for (auto entry : variable_to_index_)
        if (IsUserDefined(entry.first) or
            IsOverriddenAsUserDefined(entry.first))
          revised_simplex_solution_[entry.first] = all_sol[entry.first];

      for (auto entry : raw_variable_expression_) {
        auto raw_var = entry.first;
        auto exp = entry.second;
        while (exp.variable_coeff.size() > 0) {
          auto entry = *exp.variable_coeff.begin();
          ReplaceVariableWithExpression(exp, entry.first, all_sol[entry.first]);
        }
        revised_simplex_solution_[raw_var] = exp.constant;
      }
      return SOLVED;
    }
    List<real_t>* mu = basis_inverse->Times(tableau_->Col(entering_non_basis));

    tableau_index_t leaving_basis = -1;
    real_t min_ratio = std::numeric_limits<real_t>::max();
    for (auto iter = mu->Begin(); !iter->IsEnd(); iter = iter->Next()) {
      if (_IsNegative(iter->Data()) and
          -(basic_feasible_solution->At(iter->Index()) / iter->Data()) <
              min_ratio) {
        min_ratio = -basic_feasible_solution->At(iter->Index()) / iter->Data();
        leaving_basis = iter->Index();
      }
    }

    if (leaving_basis < 0) {
      // TODO
      return UNBOUNDED;
    }

    TableauRevisedSimplexPivot(leaving_basis, entering_non_basis, mu,
                               min_ratio);
  }
  return ERROR;
}

Num LPModel::GetTableauRevisedSimplexOptimum() {
  return revised_simplex_optimum_;
}

std::map<Variable, Num> LPModel::GetTableauRevisedSimplexSolution() {
  return revised_simplex_solution_;
}
