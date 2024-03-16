#include "lp.h"

void LPModel::RevisedSimplexPivot(int base_ind, int non_base_ind) {
  auto FindVar = [](std::set<Variable> vars, int ind) -> Variable {
    for (auto var : vars) {
      if (ind == 0) return var;
      ind -= 1;
    }
    return Variable();
  };
  Variable base = FindVar(base_variables_, base_ind),
           non_base = FindVar(non_base_variables_, non_base_ind);

  base_variables_.erase(base);
  non_base_variables_.erase(non_base);
  base_variables_.insert(non_base);
  non_base_variables_.insert(base);
}

int LPModel::RevisedSimplexFindLeavingVariable(
    RevisedSimplexMatrixForm matrix_form, int entering_ind) {
  auto entering_vec = matrix_form.non_basis_coefficient_mat.col(entering_ind);
  auto d = matrix_form.basis_coefficient_mat.inverse() * entering_vec;
  auto IsNonPositive = [](Eigen::VectorXd vec) -> bool {
    for (auto i = 0; i < vec.size(); i++) {
      if (Num(vec(i)).IsPositive()) return false;
    }
    return true;
  };
  if (IsNonPositive(d)) {
    return -1;
  }
  auto x = matrix_form.basis_coefficient_mat.inverse() * matrix_form.bound_vec;
  Num min_val = kFloatMax;
  int leaving_ind = -1;
  for (auto i = 0; i < d.size(); i++) {
    if (Num(d(i)).IsPositive()) {
      if (Num(x(i) / d(i)) < min_val) {
        min_val = Num(x(i) / d(i));
        leaving_ind = i;
      }
    }
  }
  return leaving_ind;
}

Result LPModel::RevisedSimplexSolve() {
  SlackFormSanityCheck(*this);

  // TODO: Implement the phase 1 of revised simplex method.
  auto res = Initialize();
  if (res == NOSOLUTION) return NOSOLUTION;
  assert(res == SOLVED);

  while (true) {
    auto matrix_form = ToRevisedSimplexMatrixForm();
    // Derive KKT condition.
    auto lamda = (matrix_form.basis_coefficient_mat.transpose()).inverse() *
                 matrix_form.basis_cost_vec;
    auto s_N = matrix_form.non_basis_cost_vec -
               matrix_form.non_basis_coefficient_mat.transpose() * lamda;

    auto IsNonNegative = [](Eigen::VectorXd vec) -> bool {
      for (auto i = 0; i < vec.size(); i++) {
        if (Num(vec(i)).IsNegative()) return false;
      }
      return true;
    };
    if (IsNonNegative(s_N)) {
      auto x =
          matrix_form.basis_coefficient_mat.inverse() * matrix_form.bound_vec;
      std::map<Variable, Num> all_sol;
      int i = 0;
      for (auto var : base_variables_) {
        all_sol[var] = x[i];
        if (IsUserDefined(var) or IsOverriddenAsUserDefined(var)) {
          revised_simplex_solution_[var] = x[i];
        }
        i += 1;
      }
      for (auto var : non_base_variables_) {
        all_sol[var] = kFloatZero;
        if (IsUserDefined(var) or IsOverriddenAsUserDefined(var)) {
          revised_simplex_solution_[var] = kFloatZero;
        }
      }
      for (auto entry : raw_variable_expression_) {
        auto raw_var = entry.first;
        auto exp = entry.second;
        while (exp.variable_coeff.size() > 0) {
          auto entry = *exp.variable_coeff.begin();
          ReplaceVariableWithExpression(exp, entry.first, all_sol[entry.first]);
        }
        revised_simplex_solution_[raw_var] = exp.constant;
      }
      revised_simplex_optimum_ = x.dot(matrix_form.basis_cost_vec) +
                                 model_.opt_obj.expression.constant;
      if (opt_reverted_) revised_simplex_optimum_ *= -1;
      return SOLVED;
    }
    Num min_s_N = kFloatMax;
    int entering_ind = -1;
    for (auto i = 0; i < s_N.size(); i++) {
      if (Num(s_N(i)).IsNegative() and Num(s_N(i)) < min_s_N) {
        min_s_N = Num(s_N(i));
        entering_ind = i;
      }
    }
    assert(entering_ind >= 0);
    auto leaving_ind =
        RevisedSimplexFindLeavingVariable(matrix_form, entering_ind);
    if (leaving_ind < 0) {
      return UNBOUNDED;
    }
    RevisedSimplexPivot(leaving_ind, entering_ind);
  }
  return ERROR;
}

Num LPModel::GetRevisedSimplexOptimum() { return revised_simplex_optimum_; }

std::map<Variable, Num> LPModel::GetRevisedSimplexSolution() {
  return revised_simplex_solution_;
}