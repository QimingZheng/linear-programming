/*
 * Created on Sun Jan 07 2023
 *
 * Copyright (c) 2024 - Qiming Zheng
 *
 * This file defines the interface of solving an integer linear programming
 * problem.
 *
 */
#pragma once

#include "base.h"
#include "lp.h"

class ILPModel {
 public:
  ILPModel() : opt_obj_(OptimizationObject(INTEGER)) {}

  void AddConstraint(Constraint constraint) {
    constraints_.push_back(constraint);
  }
  void SetOptimizationObject(OptimizationObject obj) {
    assert(obj.expression.constant == kIntZero);
    opt_obj_ = obj;
  }

  // Solves the integer programming problem with the cutting plane method.
  // (https://en.wikipedia.org/wiki/Cutting-plane_method)
  Result CuttingPlaneSolve();

  Constraint FindGomoryCut(LPModel &model,
                           Constraint non_integral_variable_constraint);

  std::string ToString() {
    std::string ret = "";
    ret += opt_obj_.ToString() + "\n";
    for (auto constraint : constraints_) {
      ret += constraint.ToString() + "\n";
    }
    return ret;
  }

  Num GetOptimum() { return optimum_; }

  std::map<Variable, Num> GetSolution() { return solution_; }

 private:
  std::vector<Constraint> constraints_;
  OptimizationObject opt_obj_;
  Num optimum_;
  std::map<Variable, Num> solution_;
};

// TODO: Implement the branch-and-cut method.

// TODO: Implement the branch-and-price method.
