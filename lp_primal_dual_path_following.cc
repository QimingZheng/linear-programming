#include "lp.h"

bool LPModel::IsValidPrimalDualPathFollowingInitialSolution(
    PrimalDualPathFollowingInitialSolution initial_solution) {
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

LPModel::PrimalDualPathFollowingInitialSolution
LPModel::InitializePrimalDualPathFollowingSolution(Num epsilon, Variable tao,
                                                   Variable theta,
                                                   Variable karpa) {
  auto variable_num = non_base_variables_.size();
  auto constraint_num = model_.constraints.size();

  auto matrix_form = ToMatrixForm();
  auto A = matrix_form.coefficient_mat;
  auto b = matrix_form.bound_vec;
  auto c = matrix_form.cost_vec;

  Eigen::VectorXd x = Eigen::VectorXd::Random(variable_num).cwiseAbs();
  Eigen::VectorXd s = Eigen::VectorXd::Random(variable_num).cwiseAbs();
  Eigen::VectorXd p = Eigen::VectorXd::Random(constraint_num).cwiseAbs();

  std::vector<Variable> xs;
  for (auto var : non_base_variables_) xs.push_back(var);

  auto b_bar = b - A * x;
  auto c_bar = c - A.transpose() * p - s;
  real_t z_bar = (real_t)(c.dot(x) + 1 - b.dot(p));

  OptimizationObject opt(FLOAT);
  opt.SetOptType(OptimizationObject::MIN);
  opt.expression = (1 + x.dot(s)) * theta;

  int i = 0;
  for (auto &con : model_.constraints) {
    con.expression += -b(i) * tao;
    con.expression += b_bar(i) * theta;
    con.expression.constant = kFloatZero;
    i += 1;
  }

  std::vector<Variable> dual_ps;
  std::vector<Variable> dual_ss;
  for (auto _ : model_.constraints)
    dual_ps.push_back(CreateArtificialVariable());

  for (auto _ : non_base_variables_)
    dual_ss.push_back(CreateArtificialVariable());

  i = 0;
  std::vector<Constraint> cons;
  for (auto var : non_base_variables_) {
    Constraint new_con(FLOAT);
    new_con.SetEquationType(Constraint::EQ);
    int j = 0;
    for (auto con : model_.constraints) {
      new_con.expression += -con.expression.GetCoeffOf(var) * dual_ps[j];
      j += 1;
    }
    new_con.expression += model_.opt_obj.expression.GetCoeffOf(var) * tao;
    new_con.expression += -c_bar(i) * theta;
    new_con.expression += -1.0f * dual_ss[i];
    cons.push_back(new_con);
    i += 1;
  }
  Constraint con1(FLOAT), con2(FLOAT);
  con1.SetEquationType(Constraint::EQ);
  con2.SetEquationType(Constraint::EQ);
  for (auto i = 0; i < b.size(); i++) {
    con1.expression += b(i) * dual_ps[i];
    con2.expression -= b_bar(i) * dual_ps[i];
  }
  i = 0;
  for (auto var : non_base_variables_) {
    con1.expression -= model_.opt_obj.expression.GetCoeffOf(var) * var;
    con2.expression += c_bar(i) * var;
    i += 1;
  }
  con1.expression += z_bar * theta;
  con1.expression += -1.0f * karpa;
  con2.expression += -z_bar * tao;
  con2.expression += Num(x.dot(s) + 1);
  model_.opt_obj = opt;
  for (auto con : cons) model_.constraints.push_back(con);
  model_.constraints.push_back(con1);
  model_.constraints.push_back(con2);
  non_base_variables_.insert(tao);
  non_base_variables_.insert(theta);
  non_base_variables_.insert(karpa);
  non_base_variables_.insert(dual_ps.begin(), dual_ps.end());
  non_base_variables_.insert(dual_ss.begin(), dual_ss.end());

  Eigen::VectorXd initial_x = Eigen::VectorXd::Zero(non_base_variables_.size());
  i = -1;
  for (auto var : non_base_variables_) {
    i += 1;
    if (var == theta or var == karpa or var == tao) {
      initial_x(i) = 1.0;
      continue;
    }
    auto findIndex = [](std::vector<Variable> vars, Variable v) -> int {
      int j = 0;
      for (auto x : vars) {
        if (v == x) {
          return j;
        }
        j += 1;
      }
      return -1;
    };
    auto ind = findIndex(xs, var);
    if (ind >= 0) {
      initial_x(i) = x(ind);
      continue;
    }
    ind = findIndex(dual_ps, var);
    if (ind >= 0) {
      initial_x(i) = p(ind);
      continue;
    }
    ind = findIndex(dual_ss, var);
    if (ind >= 0) {
      initial_x(i) = s(ind);
      continue;
    }
  }

  auto new_matrix_form = ToMatrixForm();

  //   assert(new_matrix_form.coefficient_mat * initial_x ==
  //          new_matrix_form.bound_vec);

  Eigen::VectorXd initial_p =
      Eigen::VectorXd::Ones(variable_num + constraint_num + 2);
  i = 0;
  int ind = 0;
  for (auto var : non_base_variables_) {
    auto contains = [](std::vector<Variable> xs, Variable var) -> bool {
      for (auto x : xs)
        if (x == var) return true;
      return false;
    };
    if (contains(xs, var) or contains(dual_ps, var)) {
      initial_p(ind) = initial_x(i);
      ind += 1;
    }
    i += 1;
  }

  return {
      initial_x, initial_p, initial_x, 1.0, 0.9, 1., 1.,
  };
}

Result LPModel::PrimalDualPathFollowingSolve(
    Num epsilon,
    LPModel::PrimalDualPathFollowingInitialSolution initial_solution) {
  assert(epsilon.float_value > 0.0f);
  Variable theta, tao, karpa;
  OptimizationObject opt(FLOAT);
  if (!IsValidPrimalDualPathFollowingInitialSolution(initial_solution)) {
    if (model_.opt_obj.opt_type == OptimizationObject::MAX) {
      model_.opt_obj.SetOptType(OptimizationObject::MIN);
      model_.opt_obj.expression *= -1.0;
      opt_reverted_ = !opt_reverted_;
    }
    opt = model_.opt_obj;
    non_base_variables_.insert(base_variables_.begin(), base_variables_.end());
    tao = CreateArtificialVariable();
    theta = CreateArtificialVariable();
    karpa = CreateArtificialVariable();
    initial_solution =
        InitializePrimalDualPathFollowingSolution(epsilon, tao, theta, karpa);
  }
  auto variable_num = non_base_variables_.size();
  auto constraint_num = model_.constraints.size();
  auto matrix_form = ToMatrixForm();
  auto A = matrix_form.coefficient_mat;
  auto c = matrix_form.cost_vec;
  auto b = matrix_form.bound_vec;
  auto e = Eigen::VectorXd::Ones(variable_num);
  auto I = e.asDiagonal().toDenseMatrix();
  auto x = initial_solution.x;
  auto p = initial_solution.p;
  auto s = initial_solution.s;
  auto mu = initial_solution.mu;
  auto alpha = initial_solution.alpha;
  auto rho = initial_solution.rho;

  auto last_result = x.dot(s);
  while (x.dot(s) >= epsilon) {
    mu = rho * x.dot(s) / variable_num;
    auto X = x.asDiagonal().toDenseMatrix();
    auto S = s.asDiagonal().toDenseMatrix();

    auto D_bar_square = X * S.inverse();

    auto D_bar = D_bar_square.sqrt();
    auto P = D_bar * A.transpose() *
             ((A * D_bar_square * A.transpose()).inverse()) * A * D_bar;
    auto v_mu = X.inverse() * D_bar * (mu * e - X * S * e);
    auto x_direction = D_bar * (I - P) * v_mu;
    auto p_direction =
        -((A * D_bar_square * A.transpose()).inverse()) * A * D_bar * v_mu;
    auto s_direction = D_bar.inverse() * P * v_mu;

    auto min_beta = [](real_t alpha, Eigen::VectorXd x,
                       Eigen::VectorXd d) -> real_t {
      real_t ret = 1000000000000.0;
      assert(x.size() == d.size());
      for (auto i = 0; i < x.size(); i++) {
        if (d(i) < 0) ret = std::min(ret, -alpha * x(i) / d(i));
      }
      return std::min(ret, 1.0);
    };
    auto beta_P = min_beta(alpha, x, x_direction);
    auto beta_D = min_beta(alpha, s, s_direction);

    x = x + beta_P * x_direction;
    p = p + beta_D * p_direction;
    s = s + beta_D * s_direction;

    if (x.dot(s) < last_result - epsilon) {
      rho = 1.0;
    } else {
      if (rho == 1.0)
        rho = 0.9;
      else
        rho = 0.9 * rho;
    }
    last_result = x.dot(s);
  }
  int i = 0;
  auto multiplier = 1.0;
  for (auto var : non_base_variables_) {
    if (var == tao) {
      multiplier = x(i);
    }
    i += 1;
  }
  i = 0;
  for (auto var : non_base_variables_) {
    if (IsUserDefined(var) or IsOverriddenAsUserDefined(var)) {
      primal_dual_path_following_solution_[var] =
          Num((real_t)x(i)) / multiplier;
    }
    i += 1;
  }
  primal_dual_path_following_optimum_ = kFloatZero;
  for (auto entry : opt.expression.variable_coeff) {
    if (primal_dual_path_following_solution_.find(entry.first) !=
        primal_dual_path_following_solution_.end())
      primal_dual_path_following_optimum_ +=
          entry.second * primal_dual_path_following_solution_[entry.first];
  }
  if (opt_reverted_) {
    primal_dual_path_following_optimum_ *= -1.0f;
  }
  return SOLVED;
}

Num LPModel::GetPrimalDualPathFollowingOptimum() {
  return primal_dual_path_following_optimum_;
}

std::map<Variable, Num> LPModel::GetPrimalDualPathFollowingSolution() {
  return primal_dual_path_following_solution_;
}