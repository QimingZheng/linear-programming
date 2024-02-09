#include "lp.h"

bool IsUserDefined(Variable var) {
  return var.variable_name.rfind(kBase, 0) != 0 and
         var.variable_name.rfind(kSubstitution, 0) != 0 and
         var.variable_name.rfind(kDual, 0) != 0 and
         var.variable_name.rfind(kArtificial, 0) != 0;
}

Variable LPModel::CreateBaseVariable() {
  auto var = Variable(kBase + std::to_string(base_variable_count_));
  base_variable_count_ += 1;
  return var;
}

Variable LPModel::CreateSubstitutionVariable() {
  auto var =
      Variable(kSubstitution + std::to_string(substitution_variable_count_));
  substitution_variable_count_ += 1;
  return var;
}

Variable LPModel::CreateDualVariable() {
  auto var = Variable(kDual + std::to_string(dual_variable_count_));
  dual_variable_count_ += 1;
  return var;
}

Variable LPModel::CreateArtificialVariable() {
  auto var = Variable(kArtificial + std::to_string(artificial_variable_count_));
  artificial_variable_count_ += 1;
  return var;
}

/*
 * There are four possible reasons that a raw linear programming problem is not
 * in a standard form:
 *    1. The objective function might be a minimization rather than a
 *       maximization.
 *    2. There might be variables without nonnegativity constraints.
 *    3. There might be equality constraints, which have an equal sign rather
 *       than a less-than-or-equal-to sign.
 *    4. There might be inequality constraints, but instead of having a
 *       less-than-or-equal-to sign, they have a greater-than-or-equal-to sign.
 */
void LPModel::ToStandardForm() {
  /* Handles the first case: negates all coefficients in the objective function.
   */
  if (model_.opt_obj.opt_type == OptimizationObject::Type::MIN) {
    opt_reverted_ = true;
    model_.opt_obj.expression *= -1.0f;
    model_.opt_obj.opt_type = OptimizationObject::Type::MAX;
  }
  /* Handles the thrid case: change a equation (lhs = rhs) into two inequations:
   * (lhs >= rhs) and (lhs <= rhs). */
  std::vector<Constraint> equation_constraint;
  for (auto& constraint : model_.constraints) {
    if (constraint.equation_type == Constraint::Type::EQ) {
      constraint.equation_type = Constraint::Type::GE;
      Constraint con(FLOAT);
      con.SetCompare(constraint.compare);
      con.SetEquationType(Constraint::Type::LE);
      con.expression = constraint.expression;
      equation_constraint.push_back(con);
    }
  }
  for (auto constrain : equation_constraint) {
    model_.constraints.push_back(constrain);
  }
  /* Handles the fourth case: negates both the lhs and rhs of the inequation,
   * change `>=` to `<=`. */
  for (auto& constraint : model_.constraints) {
    constraint.compare -= constraint.expression.constant;
    constraint.expression.constant = 0.0f;
    if (constraint.equation_type == Constraint::Type::GE) {
      constraint.expression *= -1.0f;
      constraint.compare *= -1.0f;
      constraint.equation_type = Constraint::Type::LE;
    }
  }
  /* Handles the second case: suppose xi does not have a corresponding xi >= 0
   * constraint, then we should replace all occurrence of xi by xi' - xi', and
   * add non-negativity for xi' and xi'': xi' >= 0, xi'' >= 0. */
  for (auto constraint : model_.constraints) {
    for (auto entry : constraint.expression.variable_coeff) {
      non_base_variables_.insert(entry.first);
    }
  }
  std::set<Variable> negative_vars = non_base_variables_;
  std::vector<int> non_negative_constraint_index;
  for (size_t i = 0; i < model_.constraints.size(); i++) {
    auto constraint = model_.constraints[i];
    if (!IsNonNegativeConstraint(constraint)) continue;
    auto var = constraint.expression.variable_coeff.begin()->first;
    negative_vars.erase(var);
    non_negative_variables_.insert(var);
    non_negative_constraint_index.push_back(i);
  }
  std::reverse(non_negative_constraint_index.begin(),
               non_negative_constraint_index.end());
  for (auto index : non_negative_constraint_index) {
    model_.constraints.erase(model_.constraints.begin() + index);
  }
  for (auto var : negative_vars) {
    non_base_variables_.erase(var);
    Variable s1 = CreateSubstitutionVariable(),
             s2 = CreateSubstitutionVariable();
    Expression exp = 1.0f * s1;
    exp -= 1.0f * s2;
    raw_variable_expression_.insert(std::make_pair(var, exp));
    for (auto& constraint : model_.constraints) {
      ReplaceVariableWithExpression(constraint.expression, var, exp);
    }
    ReplaceVariableWithExpression(model_.opt_obj.expression, var, exp);
    non_base_variables_.insert(s1);
    non_base_variables_.insert(s2);
    non_negative_variables_.insert(s1);
    non_negative_variables_.insert(s2);
  }
}

/* Suppose the linear programming problem has been formulated in the standard
 * form, its slack form is in the format of: s = b_i - \sum_{j=1}^{n} a_{ij} xj,
 * where s is the slack variable.
 */
void LPModel::ToSlackForm() {
  assert(StandardFormSanityCheck(*this) == true);
  for (auto& constraint : model_.constraints) {
    auto base_var = CreateBaseVariable();
    constraint.expression = constraint.compare - constraint.expression;
    constraint.compare = kFloatZero;
    constraint.equation_type = Constraint::Type::EQ;
    constraint.expression += -1.0f * base_var;
    base_variables_.insert(base_var);
  }
}

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

bool needInitialization(const std::vector<Constraint>& constraints) {
  bool needed = false;
  for (auto constraint : constraints) {
    if (constraint.expression.constant < 0.0f) {
      needed = true;
      break;
    }
  }
  return needed;
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
  //    x_{j} = b_{j} - \sum A_{j, k} x_{k} (x_{k} is non-basis) \forall j \in
  //    basis
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
    assert(constraint.expression.constant >= kFloatZero);

  // Solve the helper LP problem (can directly goes into phase 2, and the
  // problem must be solvable and bounded).
  auto result = helper_lp.SimplexSolve();
  assert(result == SOLVED);

  // If the helper LP problem's solution is nagative, which the raw LP
  // constraints is not satisfiable unless x_{0} is positive .
  if (helper_lp.GetSimplexOptimum() < kFloatZero) {
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

Result LPModel::SimplexSolve() {
  auto res = Initialize();
  if (res == NOSOLUTION) return NOSOLUTION;
  assert(res == SOLVED);
  assert(needInitialization(model_.constraints) == false);

  // Phase 2: The main optimization procedures.
  while (true) {
    Variable e;
    // Find any non-base variable x_{e} that c_e > 0.
    for (auto& entry : model_.opt_obj.expression.variable_coeff) {
      if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
          entry.second > 0.0f) {
        e = entry.first;
        break;
      }
    }
    // If not found, which means \vec c <= \vec 0, so the maximum of the
    // objective function is already achieved.
    if (e.IsUndefined()) {
      simplex_optimum_ = GetOptimum();
      simplex_solution_ = GetSolution();
      return SOLVED;
    }
    // Find a base variable x_{d} s.t. A_{d,e} > 0 and minimize b_{d}/A_{d,e}
    Variable d;
    Num min_ = kFloatMax;
    for (auto base : base_variables_) {
      for (auto constraint : model_.constraints) {
        assert(constraint.expression.constant >= kFloatZero);
        if (constraint.expression.GetCoeffOf(base) != kFloatZero and
            -constraint.expression.GetCoeffOf(e) > kFloatZero) {
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
      return UNBOUNDED;
    }
    // Perform pivot(x_{d}, x_{e})
    Pivot(d, e);
  }
  return ERROR;
}

Num LPModel::GetSimplexOptimum() { return simplex_optimum_; }

std::map<Variable, Num> LPModel::GetSimplexSolution() {
  return simplex_solution_;
}

Num LPModel::GetOptimum() {
  for (auto& entry : model_.opt_obj.expression.variable_coeff) {
    if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
        entry.second > 0.0f) {
      throw std::runtime_error("Not Solved");
    }
  }
  if (opt_reverted_) return -model_.opt_obj.expression.constant;
  return model_.opt_obj.expression.constant;
}

std::map<Variable, Num> LPModel::GetSolution() {
  std::map<Variable, Num> all_sol;
  std::map<Variable, Num> sol;
  for (auto var : non_base_variables_) {
    all_sol[var] = 0.0f;
    if (IsUserDefined(var)) sol[var] = 0.0f;
  }
  for (auto base : base_variables_) {
    for (auto constraint : model_.constraints) {
      if (constraint.expression.GetCoeffOf(base) != 0.0f) {
        all_sol[base] = -constraint.expression.constant /
                        constraint.expression.GetCoeffOf(base);
        if (IsUserDefined(base))
          sol[base] = -constraint.expression.constant /
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

bool StandardFormSanityCheck(LPModel model) {
  if (model.model_.opt_obj.opt_type != OptimizationObject::Type::MAX)
    return false;
  for (auto constraint : model.model_.constraints) {
    if (constraint.equation_type != Constraint::Type::LE) return false;
  }
  for (auto var : model.non_base_variables_) {
    if (model.non_negative_variables_.find(var) ==
        model.non_negative_variables_.end())
      return false;
  }
  return true;
}

bool SlackFormSanityCheck(LPModel model) {
  std::set<Variable> appeared_vars;
  for (auto& constraint : model.model_.constraints) {
    if (constraint.equation_type != Constraint::Type::EQ) return false;
    int base_vars = 0;
    for (auto& entry : constraint.expression.variable_coeff) {
      if (model.base_variables_.find(entry.first) !=
          model.base_variables_.end()) {
        if (appeared_vars.find(entry.first) != appeared_vars.end())
          return false;
        appeared_vars.insert(entry.first);
        if (entry.second != -1.0f) return false;
        base_vars += 1;
      }
    }
    if (base_vars != 1) return false;
  }
  return true;
}

bool LPModel::IsNonNegativeConstraint(const Constraint& constraint) {
  if (constraint.equation_type != Constraint::Type::LE) return false;
  if (constraint.compare != kFloatZero) return false;
  if (constraint.expression.constant != kFloatZero) return false;
  if (constraint.expression.variable_coeff.size() != 1) return false;
  if (constraint.expression.variable_coeff.begin()->second != Num(-1.0f))
    return false;
  return true;
}

LPModel LPModel::ToDualForm() {
  // The primal LP should be in standard form:
  //  max c^T x
  //  s.t.
  //    Ax <= b
  //    x >= 0
  assert(StandardFormSanityCheck(*this) == true);
  LPModel dual;
  // The dual LP:
  //  min b^T y
  //  s.t.
  //    A^T y >= c
  //    y >= 0
  std::vector<Variable> ys;
  for (auto con : model_.constraints) {
    Variable y = CreateDualVariable();
    ys.push_back(y);
    dual.model_.opt_obj.expression += con.compare * y;
  }
  dual.model_.opt_obj.opt_type = OptimizationObject::Type::MIN;
  for (auto variable : non_base_variables_) {
    Constraint con(FLOAT);
    con.equation_type = Constraint::Type::GE;
    con.compare = model_.opt_obj.expression.GetCoeffOf(variable);
    for (size_t i = 0; i < model_.constraints.size(); i++) {
      con.expression +=
          model_.constraints[i].expression.GetCoeffOf(variable) * ys[i];
    }
    dual.AddConstraint(con);
  }
  for (auto y : ys) {
    Constraint con(FLOAT);
    con.equation_type = Constraint::Type::GE;
    con.expression = 1.0f * y;
    dual.AddConstraint(con);
  }
  return dual;
}

void LPModel::GaussianElimination(std::set<Variable> base_variables) {
  for (auto con : model_.constraints) {
    assert(con.compare == kFloatZero);
    assert(con.equation_type == Constraint::Type::EQ);
  }
  for (auto base : base_variables) {
    for (size_t i = 0; i < model_.constraints.size(); i++) {
      Constraint& con = model_.constraints[i];
      if (con.expression.GetCoeffOf(base) == kFloatZero) continue;
      con.expression *= (1.0f / con.expression.GetCoeffOf(base));
      for (auto& c : model_.constraints) {
        if (c == con) continue;
        c.expression -= (con.expression * c.expression.GetCoeffOf(base));
      }
      break;
    }
  }
}

Result LPModel::DualSolve(std::set<Variable> dual_feasible_solution_basis) {
  // Dual solve requires the optimization objective function in minimization
  // form.
  if (model_.opt_obj.opt_type == OptimizationObject::MAX) {
    for (auto& entry : model_.opt_obj.expression.variable_coeff) {
      entry.second *= -1.0f;
    }
    model_.opt_obj.SetOptType(OptimizationObject::MIN);
    opt_reverted_ = !opt_reverted_;
  }
  for (auto base : base_variables_) non_base_variables_.insert(base);
  base_variables_ = dual_feasible_solution_basis;
  for (auto base : base_variables_) {
    if (non_base_variables_.find(base) != non_base_variables_.end())
      non_base_variables_.erase(base);
  }
  // Perform Gaussian eklimination to transform the simplex tableu into:
  //  B^{-1} A = b
  GaussianElimination(dual_feasible_solution_basis);
  while (true) {
    bool found_optimal = true;
    Variable b;
    Num val = kFloatZero;
    Constraint c(FLOAT);
    // The primal optimum is achieved if x_{b} >= 0 for all base variable x_{b}.
    // Otherwise, find the minimum x_{r} that s.t. x_{r} < 0, and make it leave
    // the base.
    for (auto con : model_.constraints) {
      for (auto base : base_variables_) {
        if (con.expression.GetCoeffOf(base) != kFloatZero) {
          assert(con.expression.GetCoeffOf(base) == kFloatOne);
          if (val >
              -con.expression.constant / con.expression.GetCoeffOf(base)) {
            found_optimal = false;
            b = base;
            val = -con.expression.constant / con.expression.GetCoeffOf(base);
            c = con;
          }
        }
      }
    }
    if (found_optimal) break;

    // Find the non-base variable x_{j} that minimize:
    //    -(c_{j} - c_{B}^{T} B^{-1} A)/A_{rj} (where A_{rj} < 0)
    val = kFloatMax;
    Variable n;
    for (auto entry : c.expression.variable_coeff) {
      if (entry.first == b) continue;
      if (entry.second < kFloatZero) {
        Num r = model_.opt_obj.expression.GetCoeffOf(entry.first);
        for (auto e : base_variables_) {
          for (auto con : model_.constraints) {
            if (con.expression.GetCoeffOf(e) != kFloatZero) {
              r -= model_.opt_obj.expression.GetCoeffOf(e) *
                   con.expression.GetCoeffOf(entry.first);
            }
          }
        }
        if ((r / (-entry.second)) < val) {
          val = r / (-entry.second);
          n = entry.first;
        }
      }
    }
    if (n.IsUndefined()) return UNBOUNDED;

    // Perform enter base operation for x_{j}, leave base operation for x_{r}.
    for (auto& con : model_.constraints) {
      if (con == c) {
        con.expression *= 1.0f / c.expression.GetCoeffOf(n);
        c = con;
        break;
      }
    }
    for (auto& con : model_.constraints) {
      if (con == c) continue;
      con.expression -= con.expression.GetCoeffOf(n) * c.expression;
    }
    base_variables_.erase(b);
    base_variables_.insert(n);
    non_base_variables_.erase(n);
    non_base_variables_.insert(b);
  }
  return SOLVED;
}

Num LPModel::GetDualSolveOptimum() {
  auto sol = GetSolution();
  Num ret(FLOAT);
  for (auto entry : sol) {
    ret += entry.second * model_.opt_obj.expression.GetCoeffOf(entry.first);
  }
  if (opt_reverted_) ret = -ret;
  return ret;
}

std::map<Variable, Num> LPModel::GetDualSolveSolution() {
  return GetSolution();
}

Result LPModel::ColumnGenerationSolve() {
  LPModel master_problem;
  // Initialize the initial master problem by adding artificial variables to set
  // up the initial restricted master problem:
  //    max c^T x - \infinity^T y
  //    s.t. Ax +/- y <= b (`+` if (b >= 0) else `-`)
  //      x, y >= 0
  // Then a trivial feasible solution to the raw problem is: x = 0, y = |b|
  std::set<Variable> artificials;
  for (size_t i = 0; i < model_.constraints.size(); i++) {
    auto artificial = CreateArtificialVariable();
    artificials.insert(artificial);
    non_base_variables_.insert(artificial);
    if (model_.constraints[i].compare >= kFloatZero) {
      model_.constraints[i].expression += 1.0f * artificial;
    } else {
      model_.constraints[i].expression += -1.0f * artificial;
    }
    Constraint constraint(FLOAT);
    constraint.equation_type = Constraint::LE;
    constraint.expression =
        model_.constraints[i].expression.GetCoeffOf(artificial) * artificial;
    constraint.compare = model_.constraints[i].compare;
    master_problem.AddConstraint(constraint);
  }
  master_problem.opt_reverted_ = opt_reverted_;
  OptimizationObject opt(FLOAT);
  opt.opt_type = OptimizationObject::MAX;
  for (auto artificial : artificials) {
    model_.opt_obj.expression += -1000000000.0f * artificial;
    opt.expression += -1000000000.0f * artificial;
  }
  master_problem.SetOptimizationObject(opt);
  std::set<Variable> added_variables = artificials;
  master_problem.non_base_variables_ = artificials;
  master_problem.non_negative_variables_ = artificials;
  while (true) {
    // Step 1: solve the dual problem:
    LPModel dual_problem = master_problem.ToDualForm();
    dual_problem.ToStandardForm();
    dual_problem.ToSlackForm();
    auto result = dual_problem.SimplexSolve();
    if (result == NOSOLUTION) return UNBOUNDED;
    if (result == UNBOUNDED) return NOSOLUTION;
    assert(result == SOLVED);
    auto sol = dual_problem.GetSimplexSolution();
    // Pricing problem: Find a non-base x_{j} with maximized reduced cost,
    // where the pricing objective function is: c_{j} - u^T b (u is the solution
    // to the dual LP).
    std::set<Variable> all_vars = non_base_variables_;
    Variable to_be_added;
    Num val = Num(-1000000000.f);
    for (auto var : all_vars) {
      if (added_variables.find(var) != added_variables.end()) continue;
      auto c = model_.opt_obj.expression.GetCoeffOf(var);
      Num u = kFloatZero;
      int i = 0;
      for (auto entry : sol) {
        u += model_.constraints[i].expression.GetCoeffOf(var) * entry.second;
        i += 1;
      }
      if (val < c - u) {
        val = c - u;
        to_be_added = var;
      }
    }
    // If no improving variables can be found, then the optimum is achieved.
    if (to_be_added.IsUndefined()) break;
    // Add the improving variable to the master problem (both objective function
    // and constraints).
    for (size_t i = 0; i < model_.constraints.size(); i++) {
      auto con = model_.constraints[i];
      master_problem.model_.constraints[i].expression +=
          con.expression.GetCoeffOf(to_be_added) * to_be_added;
    }
    master_problem.model_.opt_obj.expression +=
        model_.opt_obj.expression.GetCoeffOf(to_be_added) * to_be_added;
    master_problem.non_base_variables_.insert(to_be_added);
    master_problem.non_negative_variables_.insert(to_be_added);
    added_variables.insert(to_be_added);
  }
  for (auto art : artificials) {
    master_problem.model_.opt_obj.expression.SetCoeffOf(art, kFloatZero);
    for (auto& con : master_problem.model_.constraints) {
      con.expression.SetCoeffOf(art, kFloatZero);
    }
  }
  master_problem.ToSlackForm();
  master_problem.SimplexSolve();
  column_generation_optimum_ = master_problem.GetSimplexOptimum();
  column_generation_solution_ = master_problem.GetSimplexSolution();
  return SOLVED;
}

Num LPModel::GetColumnGenerationOptimum() { return column_generation_optimum_; }

std::map<Variable, Num> LPModel::GetColumnGenerationSolution() {
  return column_generation_solution_;
}