#include "lp.h"

bool IsUserDefined(Variable var) {
  return var.variable_name.rfind(kBase, 0) != 0 and
         var.variable_name.rfind(kSubstitution, 0) != 0 and
         var.variable_name.rfind(kDual, 0) != 0 and
         var.variable_name.rfind(kArtificial, 0) != 0;
}

void LPModel::OverrideAsUserDefined(Variable var) {
  overrided_as_user_defined_variables_.insert(var);
}

bool LPModel::IsOverriddenAsUserDefined(Variable var) {
  return overrided_as_user_defined_variables_.find(var) !=
         overrided_as_user_defined_variables_.end();
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

Num LPModel::GetOptimum(bool check_optimal_condition) {
  if (check_optimal_condition) {
    for (auto& entry : model_.opt_obj.expression.variable_coeff) {
      if (non_base_variables_.find(entry.first) != non_base_variables_.end() and
          entry.second > 0.0f) {
        throw std::runtime_error("Not Solved");
      }
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
    if (IsUserDefined(var) or IsOverriddenAsUserDefined(var)) sol[var] = 0.0f;
  }
  for (auto base : base_variables_) {
    for (auto constraint : model_.constraints) {
      if (constraint.expression.GetCoeffOf(base) != 0.0f) {
        all_sol[base] = -constraint.expression.constant /
                        constraint.expression.GetCoeffOf(base);
        if (IsUserDefined(base) or IsOverriddenAsUserDefined(base))
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
  if (!constraint.compare.IsZero()) return false;
  if (!constraint.expression.constant.IsZero()) return false;
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
    dual.OverrideAsUserDefined(y);
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

void LPModel::LogIterStatus(int iter, long delta, real_t optimum) {
  std::cout << "[Iter " + std::to_string(iter) + "]: "
            << std::to_string(optimum) << " in " << std::to_string(delta / 1000)
            << "ms\n";
}

// Convert the constraints to its matrix form.
Eigen::MatrixXd LPModel::ToMatrixForm() {
  int variable_num = non_base_variables_.size();
  int constraint_num = model_.constraints.size();
  Eigen::MatrixXd A(constraint_num, variable_num);
  int col = 0;
  for (auto var : non_base_variables_) {
    int row = 0;
    for (auto con : model_.constraints) {
      A(row, col) = con.expression.GetCoeffOf(var).float_value;
      row += 1;
    }
    col += 1;
  }
  return A;
}