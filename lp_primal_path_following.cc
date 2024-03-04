#include "lp.h"

bool LPModel::IsValidPrimalPathFollowingInitialSolution(
    PrimalPathFollowingInitialSolution initial_solution) {
  if (initial_solution.x.size() == 0) return false;
  if (initial_solution.p.size() == 0) return false;
  if (initial_solution.s.size() == 0) return false;
  for (size_t i = 0; i < initial_solution.x.size(); i++) {
    if (initial_solution.x(i) < 0) return false;
  }
  for (size_t i = 0; i < initial_solution.s.size(); i++) {
    if (initial_solution.s(i) < 0) return false;
  }
  if (initial_solution.mu <= 0.0) return false;
  if (initial_solution.alpha < 0.0 or initial_solution.alpha > 1.0)
    return false;
  return true;
}

LPModel::PrimalPathFollowingInitialSolution
LPModel::InitializePrimalPathFollowingSolution(Num epsilon, Variable x1,
                                               Variable x2) {
  auto findMaxAbs = [](Expression exp) -> real_t {
    real_t max_abs = 0.0f;
    for (auto entry : exp.variable_coeff) {
      if (std::abs(entry.second.float_value) > max_abs)
        max_abs = std::abs(entry.second.float_value);
    }
    return max_abs;
  };

  // Find the maximum absolute value of constants in the LP model.
  auto U = 0.0;
  for (auto con : model_.constraints) {
    U = std::max(U, findMaxAbs(con.expression));
    U = std::max(U, std::abs(con.expression.constant.float_value));
  }
  U = std::max(U, findMaxAbs(model_.opt_obj.expression));
  U = std::max(U, std::abs(model_.opt_obj.expression.constant.float_value));

  auto M = 10000.0 * U;

  non_base_variables_.insert(x1);
  non_base_variables_.insert(x2);
  model_.opt_obj.expression += M * x1;

  int variable_num = non_base_variables_.size();
  int constraint_num = model_.constraints.size();

  for (auto &con : model_.constraints) {
    con.expression.constant *=
        variable_num * 1.0 /
        ((variable_num - 2) * std::pow(constraint_num * U, constraint_num));
    auto sum = [](Constraint con) -> real_t {
      real_t ret = 0;
      for (auto entry : con.expression.variable_coeff) {
        ret += entry.second.float_value;
      }
      return ret;
    };
    con.expression += ((-con.expression.constant) - sum(con)) * x1;
  }

  Constraint con(FLOAT);
  con.SetEquationType(Constraint::EQ);
  for (auto var : non_base_variables_) {
    con.expression += var;
  }
  con.SetConstant(-1.0 * variable_num);
  AddConstraint(con);
  constraint_num += 1;

  auto square_sum = [](Expression exp) -> real_t {
    real_t ret = 0.0;
    for (auto entry : exp.variable_coeff) {
      ret += (entry.second * entry.second).float_value;
    }
    return ret;
  };
  auto mu = 4.0 * std::sqrt(square_sum(model_.opt_obj.expression));
  Eigen::VectorXd p = Eigen::VectorXd::Zero(constraint_num);
  p(constraint_num - 1) = -mu;
  Eigen::VectorXd s(variable_num);
  int i = 0;
  for (auto var : non_base_variables_) {
    s(i) = model_.opt_obj.expression.GetCoeffOf(var).float_value + mu;
    i += 1;
  }
  constraint_num -= 1;
  PrimalPathFollowingInitialSolution initial_solution = {
      Eigen::VectorXd::Ones(variable_num),
      p,
      s,
      mu,
      1.0 - (0.25) / (0.5 + std::sqrt(variable_num)),
      ((variable_num - 2) * std::pow(constraint_num * U, constraint_num)) /
          (variable_num * 1.0),
  };
  return initial_solution;
}

Result LPModel::PrimalPathFollowingSolve(
    Num epsilon, PrimalPathFollowingInitialSolution initial_solution) {
  non_base_variables_.insert(base_variables_.begin(), base_variables_.end());
  Variable x1, x2;
  if (!IsValidPrimalPathFollowingInitialSolution(initial_solution)) {
    // Check the formulation:
    if (model_.opt_obj.opt_type == OptimizationObject::MAX) {
      model_.opt_obj.SetOptType(OptimizationObject::MIN);
      model_.opt_obj.expression *= -1.0;
      opt_reverted_ = !opt_reverted_;
    }
    x1 = CreateArtificialVariable();
    x2 = CreateArtificialVariable();
    initial_solution = InitializePrimalPathFollowingSolution(epsilon, x1, x2);
  }
  int variable_num = non_base_variables_.size();
  int constraint_num = model_.constraints.size();
  auto matrix_form = ToMatrixForm();
  Eigen::MatrixXd A = matrix_form.coefficient_mat;
  Eigen::VectorXd e = Eigen::VectorXd::Ones(variable_num);
  Eigen::VectorXd c = matrix_form.cost_vec;
  Eigen::MatrixXd I = e.asDiagonal().toDenseMatrix();
  auto x = initial_solution.x;
  auto s = initial_solution.s;
  auto p = initial_solution.p;
  assert(x.size() == s.size());
  real_t mu = initial_solution.mu;
  real_t alpha = initial_solution.alpha;
  while (x.dot(s) >= epsilon) {
    auto X = x.asDiagonal().toDenseMatrix();
    auto X_square = X * X;
    mu = alpha * mu;
    auto newton_direction =
        (I - X_square * A.transpose() *
                 (A * X_square * A.transpose()).inverse() * A) *
        (X * e - (1.0 / mu) * X_square * c);
    auto newton_step = (A * X_square * A.transpose()).inverse() * A *
                       (X_square * c - mu * X * e);
    assert(x.size() == newton_direction.size());
    assert(p.size() == newton_step.size());
    x = x + newton_direction;
    p = newton_step;
    s = c - A.transpose() * newton_step;
  }
  int i = 0;
  for (auto var : non_base_variables_) {
    if (IsUserDefined(var) or IsOverriddenAsUserDefined(var))
      primal_path_following_solution_[var] = x(i) * initial_solution.multiplier;
    i += 1;
  }
  primal_path_following_optimum_ = kFloatZero;
  for (auto entry : model_.opt_obj.expression.variable_coeff) {
    if (entry.first == x1 or entry.second == x2) continue;
    primal_path_following_optimum_ +=
        entry.second * primal_path_following_solution_[entry.first];
  }
  if (opt_reverted_) {
    primal_path_following_optimum_ *= -1.0f;
  }
  return SOLVED;
}

Num LPModel::GetPrimalPathFollowingOptimum() {
  return primal_path_following_optimum_;
}

std::map<Variable, Num> LPModel::GetPrimalPathFollowingSolution() {
  return primal_path_following_solution_;
}
