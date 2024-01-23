#include "lp.h"

int LPModel::base_variable_count_ = 0;
int LPModel::substitution_variable_count_ = 0;
int LPModel::dual_variable_count_ = 0;
int LPModel::artificial_variable_count_ = 0;

bool IsUserDefined(Variable var) {
  return var.variable_name.rfind(kBase, 0) != 0 &&
         var.variable_name.rfind(kSubstitution, 0) != 0;
}

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

Variable CreateDualVariable() {
  auto var = Variable(kDual + std::to_string(LPModel::dual_variable_count_));
  LPModel::dual_variable_count_ += 1;
  return var;
}

Variable CreateArtificialVariable() {
  auto var = Variable(kArtificial +
                      std::to_string(LPModel::artificial_variable_count_));
  LPModel::artificial_variable_count_ += 1;
  return var;
}

/*
 * There four possible reasons that a raw linear programming problem is not in a
 * standard form:
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
  for (int i = 0; i < model_.constraints.size(); i++) {
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
  for (auto& constraint : model_.constraints) {
    auto base_var = CreateBaseVariable();
    constraint.expression = constraint.compare - constraint.expression;
    constraint.compare = 0.0f;
    constraint.equation_type = Constraint::Type::EQ;
    constraint.expression += -1.0f * base_var;
    base_variables_.insert(base_var);
  }
}

void LPModel::Pivot(Variable base, Variable non_base) {
  assert(non_base_variables_.find(non_base) != non_base_variables_.end());
  assert(base_variables_.find(base) != base_variables_.end());
  Expression substitution(0.0f);
  for (auto& constraint : model_.constraints) {
    if (constraint.expression.GetCoeffOf(base) != 0.0f and
        constraint.expression.GetCoeffOf(non_base) != 0.0f) {
      auto coeff_of_non_base = constraint.expression.GetCoeffOf(non_base);
      substitution = constraint.expression;
      substitution.SetCoeffOf(non_base, 0.0f);
      substitution /= (-coeff_of_non_base);
      base_variables_.erase(base);
      base_variables_.insert(non_base);
      non_base_variables_.erase(non_base);
      non_base_variables_.insert(base);
      break;
    }
  }
  ReplaceVariableWithExpression(model_.opt_obj.expression, non_base,
                                substitution);
  for (auto& constraint : model_.constraints) {
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

Result LPModel::Initialize() {
  // Phase 1: If there are negative constants in some constraint, use Pivot
  // method to transform the constrain system to non-negative form.
  bool need_transform = false;
  for (auto constraint : model_.constraints) {
    if (constraint.expression.constant < 0.0f) {
      need_transform = true;
    }
  }
  if (!need_transform) return SOLVED;
  LPModel phase1_model;
  Variable artificial_var("artificial_variable");
  for (auto constraint : model_.constraints) {
    constraint.expression += artificial_var;
    phase1_model.AddConstraint(constraint);
  }
  phase1_model.non_base_variables_ = non_base_variables_;
  phase1_model.non_base_variables_.insert(artificial_var);
  phase1_model.base_variables_ = base_variables_;
  OptimizationObject obj(FLOAT);
  obj.expression += -1.0f * artificial_var;
  obj.SetOptType(OptimizationObject::MAX);
  phase1_model.SetOptimizationObject(obj);
  Num minimum = 0.0f;
  int best = -1;
  for (int i = 0; i < phase1_model.model_.constraints.size(); i++) {
    auto constraint = phase1_model.model_.constraints[i];
    if (constraint.expression.constant < minimum) {
      minimum = constraint.expression.constant;
      best = i;
    }
  }
  assert(best >= 0);
  for (auto entry :
       phase1_model.model_.constraints[best].expression.variable_coeff) {
    auto var = entry.first;
    if (base_variables_.find(var) != base_variables_.end()) {
      phase1_model.Pivot(var, artificial_var);
    }
  }
  for (auto constraint : phase1_model.model_.constraints) {
    assert(constraint.expression.constant >= 0.0f);
  }
  phase1_model.Solve();
  if (phase1_model.GetOptimum() < 0.0f) {
    return NOSOLUTION;
  }
  if (phase1_model.base_variables_.find(artificial_var) !=
      phase1_model.base_variables_.end()) {
    auto any_non_base_var = phase1_model.non_base_variables_.begin();
    phase1_model.Pivot(artificial_var, *any_non_base_var);
  }
  assert(phase1_model.non_base_variables_.find(artificial_var) !=
         phase1_model.non_base_variables_.end());
  for (auto& constraint : phase1_model.model_.constraints) {
    constraint.expression.SetCoeffOf(artificial_var, 0.0f);
  }
  model_.constraints = phase1_model.model_.constraints;
  phase1_model.non_base_variables_.erase(artificial_var);
  base_variables_ = phase1_model.base_variables_;
  non_base_variables_ = phase1_model.non_base_variables_;
  for (auto base : phase1_model.base_variables_) {
    Expression substitution(0.0f);
    for (auto constraint : phase1_model.model_.constraints) {
      if (constraint.expression.GetCoeffOf(base) != 0.0f) {
        substitution = constraint.expression;
        substitution *= (-1.0f / constraint.expression.GetCoeffOf(base));
        substitution.SetCoeffOf(base, 0.0f);
        break;
      }
    }
    ReplaceVariableWithExpression(model_.opt_obj.expression, base,
                                  substitution);
  }
  return SOLVED;
}

Result LPModel::Solve() {
  auto res = Initialize();
  if (res == NOSOLUTION) return NOSOLUTION;
  for (auto constraint : model_.constraints) {
    assert(constraint.expression.constant >= 0.0f);
  }
  // Phase 2: The main optimization procedures.
  while (true) {
    Variable e;
    for (auto& entry : model_.opt_obj.expression.variable_coeff) {
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
      for (auto constraint : model_.constraints) {
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
    for (int i = 0; i < model_.constraints.size(); i++) {
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
    for (int i = 0; i < model_.constraints.size(); i++) {
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
  // form:
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
  GaussianElimination(dual_feasible_solution_basis);
  while (true) {
    bool found_optimal = true;
    Variable b;
    Num val = kFloatZero;
    Constraint c(FLOAT);
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

    val = Num(1000000000000.0f);
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

Result LPModel::ColumnGenerationSolve() {
  LPModel master_problem;
  // Add artificial variables to set up the initial restricted master problem.
  std::set<Variable> artificials;
  for (int i = 0; i < model_.constraints.size(); i++) {
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
    LPModel dual_problem = master_problem.ToDualForm();
    dual_problem.ToStandardForm();
    dual_problem.ToSlackForm();
    auto result = dual_problem.Solve();
    if (result == NOSOLUTION) return UNBOUNDED;
    if (result == UNBOUNDED) return NOSOLUTION;
    assert(result == SOLVED);
    auto sol = dual_problem.GetSolution();
    // Pricing problem.
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
    if (to_be_added.IsUndefined()) break;
    if (val <= 0.0f) break;
    for (int i = 0; i < model_.constraints.size(); i++) {
      auto con = model_.constraints[i];
      master_problem.model_.constraints[i].expression +=
          con.expression.GetCoeffOf(to_be_added) * to_be_added;
    }
    master_problem.model_.opt_obj.expression +=
        model_.opt_obj.expression.GetCoeffOf(to_be_added) * to_be_added;
    master_problem.non_base_variables_.insert(to_be_added);
    master_problem.non_negative_variables_.insert(to_be_added);
  }
  for (auto art : artificials) {
    master_problem.model_.opt_obj.expression.SetCoeffOf(art, kFloatZero);
    for (auto& con : master_problem.model_.constraints) {
      con.expression.SetCoeffOf(art, kFloatZero);
    }
  }
  master_problem.ToSlackForm();
  master_problem.Solve();
  column_generation_optimum_ = master_problem.GetOptimum();
  column_generation_solution_ = master_problem.GetSolution();
  return SOLVED;
}

Num LPModel::GetColumnGenerationOptimum() { return column_generation_optimum_; }

std::map<Variable, Num> LPModel::GetColumnGenerationSolution() {
  return column_generation_solution_;
}