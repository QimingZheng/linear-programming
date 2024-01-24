/*
 * Created on Sun Jan 07 2024
 *
 * Copyright (c) 2024 - Qiming Zheng
 *
 * This file defines the interface of solving an integer linear programming
 * problem.
 *
 */
#pragma once

#include <queue>

#include "base.h"
#include "lp.h"

class ILPModel {
 public:
  ILPModel(Model model) : model_(model) {}
  ILPModel() : model_({{}, OptimizationObject(INTEGER)}) {}

  void AddConstraint(Constraint constraint) {
    model_.constraints.push_back(constraint);
  }
  void SetOptimizationObject(OptimizationObject obj) {
    // assert(obj.expression.constant == kIntZero);
    model_.opt_obj = obj;
  }

  // Solves the integer programming problem with the branch and cut method.
  // (https://en.wikipedia.org/wiki/Branch_and_bound)
  Result BranchAndBoundSolve();

  // Solves the integer programming problem with the cutting plane method.
  // (https://en.wikipedia.org/wiki/Cutting-plane_method)
  Result CuttingPlaneSolve();

  Constraint FindGomoryCut(LPModel &model,
                           Constraint non_integral_variable_constraint);

  LPModel ToRelaxedLPModel();

  std::string ToString() {
    std::string ret = "";
    ret += model_.opt_obj.ToString() + "\n";
    for (auto constraint : model_.constraints) {
      ret += constraint.ToString() + "\n";
    }
    return ret;
  }

  Num GetOptimum() { return optimum_; }

  std::map<Variable, Num> GetSolution() { return solution_; }

 private:
  Model model_;
  // std::vector<Constraint> constraints_;
  // OptimizationObject opt_obj_;
  Num optimum_;
  std::map<Variable, Num> solution_;
};

// TODO: Implement the branch-and-cut method.

// TODO: Implement the branch-and-price method.
