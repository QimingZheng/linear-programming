#include "lp.h"

void LPModel::GaussianElimination(std::set<Variable> base_variables) {
  for (auto con : model_.constraints) {
    assert(con.compare.IsZero());
    assert(con.equation_type == Constraint::Type::EQ);
  }
  for (auto base : base_variables) {
    for (size_t i = 0; i < model_.constraints.size(); i++) {
      Constraint& con = model_.constraints[i];
      if (con.expression.GetCoeffOf(base).IsZero()) continue;
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
        if (!con.expression.GetCoeffOf(base).IsZero()) {
          assert(con.expression.GetCoeffOf(base).IsOne());
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
      if (entry.second.IsNegative()) {
        Num r = model_.opt_obj.expression.GetCoeffOf(entry.first);
        for (auto e : base_variables_) {
          for (auto con : model_.constraints) {
            if (!con.expression.GetCoeffOf(e).IsZero()) {
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
