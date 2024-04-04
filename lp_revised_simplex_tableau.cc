#include "lp.h"

template <>
inline bool _IsZeroT(const real_t& x) {
  return std::abs(x) < kEpsilonF;
}

Result LPModel::TableauRevisiedSimplexSolve() {
  assert(model_.opt_obj.opt_type == OptimizationObject::MAX);
  opt_obj_tableau_->Scale(-1.0);
  opt_reverted_ = !opt_reverted_;
  assert(base_variables_.size() == tableau_->Rows());
  tableau_size_t basis_number = base_variables_.size();
  List<real_t>* basic_feasible_solution = new List<real_t>(basis_number, DENSE);
  List<real_t>* basis_coeff = new List<real_t>(basis_number, DENSE);
  Tableau<real_t>* basis_inverse =
      new Tableau<real_t>(basis_number, basis_number, ROW_ONLY);
  tableau_index_t* basis_indices = new tableau_index_t[basis_number];
  int i = 0;
  std::vector<int> col_of_each_row(basis_number);
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
        col_of_each_row[iter->Index()] = i;
        initialized = true;
        break;
      }
    }
    assert(initialized);
    i++;
  }
  i = 0;
  for (auto x : col_of_each_row) {
    List<real_t>* basis_inverse_row = new List<real_t>(0, SPARSE);
    basis_inverse_row->Append(x, 1.);
    basis_inverse->AppendRow(i, basis_inverse_row);
    i++;
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

    tableau_is_base_variable_[entering_non_basis] = true;
    tableau_is_base_variable_[basis_indices[leaving_basis]] = false;
    basic_feasible_solution->AddScaled(mu, min_ratio, true);
    basic_feasible_solution->Set(leaving_basis, min_ratio);
    basis_indices[leaving_basis] = entering_non_basis;
    basis_coeff->Set(leaving_basis, opt_obj_tableau_->At(entering_non_basis));

    basis_inverse->Row(leaving_basis)->Scale(-1.0 / mu->At(leaving_basis));
    mu->Set(leaving_basis, 0);
    auto helper_tableau =
        mu->SparseCross(basis_inverse->Row(leaving_basis), ROW_ONLY);
    basis_inverse->Add(helper_tableau);
  }
  return ERROR;
}

Num LPModel::GetTableauRevisedSimplexOptimum() {
  return revised_simplex_optimum_;
}

std::map<Variable, Num> LPModel::GetTableauRevisedSimplexSolution() {
  return revised_simplex_solution_;
}
