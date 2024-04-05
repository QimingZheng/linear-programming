#include <fstream>
#include <sstream>

#include "ilp.h"
#include "lp.h"
#include "parser.h"

enum SolverAlgorithm {
  SOLVER_UNKNOWN,
  SIMPLEX,
  SIMPLEX_TABLEAU,
  REVISED_SIMPLEX_TABLEAU,
  DUAL_SIMPLEX,
  COLUMN_GENERATION,
};

std::string ToLower(std::string str) {
  std::string ret = "";
  for (auto s : str) {
    ret += std::tolower(s);
  }
  return ret;
}

SolverAlgorithm ParseAlgorithm(std::string algo) {
  if (ToLower(algo) == "simplex") {
    return SIMPLEX;
  }
  if (ToLower(algo) == "simplex_tableau") {
    return SIMPLEX_TABLEAU;
  }
  if (ToLower(algo) == "dual_simplex") {
    return DUAL_SIMPLEX;
  }
  if (ToLower(algo) == "column_generation") {
    return COLUMN_GENERATION;
  }
  if (ToLower(algo) == "revised_simplex_tableau") {
    return REVISED_SIMPLEX_TABLEAU;
  }
  return SOLVER_UNKNOWN;
}

int main(int argc, char **argv) {
  assert(int(-1.5) == -1);
  assert(int(1.5) == 1);
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " input-file [solver-algo]\n";
    return -1;
  }
  std::ifstream lpfile(argv[1]);
  SolverAlgorithm solver = SIMPLEX;
  if (argc == 3) solver = ParseAlgorithm(argv[2]);
  Parser parser;
  Model model = parser.Parse(lpfile);
  auto hasFloatVar = [](Constraint constraint) {
    for (auto entry : constraint.expression.variable_coeff)
      if (entry.first.type == FLOAT) return true;
    return false;
  };
  auto isIlp = [&](Model model) {
    for (auto con : model.constraints)
      if (hasFloatVar(con)) return false;
    return true;
  };

  LPModel lp_model;
  ILPModel ilp_model;
  bool is_ilp = isIlp(model);
  if (is_ilp) {
    ilp_model = model;
  } else {
    lp_model = model;
  }
  if (is_ilp) {
    auto res = ilp_model.CuttingPlaneSolve();
    if (res == Result::SOLVED) {
      std::cout << ilp_model.GetOptimum().ToString() << "\n";
      for (auto entry : ilp_model.GetSolution()) {
        std::cout << entry.first.ToString() << " = " << entry.second.ToString()
                  << "\n";
      }
    }
  } else {
    Result result;
    Num optimum;
    std::map<Variable, Num> solution;
    switch (solver) {
      case SIMPLEX: {
        lp_model.ToStandardForm();
        lp_model.ToSlackForm();
        result = lp_model.SimplexSolve();
        if (result == Result::SOLVED) {
          optimum = lp_model.GetSimplexOptimum();
          solution = lp_model.GetSimplexSolution();
        }
      } break;

      case SIMPLEX_TABLEAU: {
        lp_model.ToStandardForm();
        lp_model.ToSlackForm();
        lp_model.ToTableau();
        result = lp_model.TableauSimplexSolve();
        if (result == Result::SOLVED) {
          optimum = lp_model.GetTableauSimplexOptimum();
          solution = lp_model.GetTableauSimplexSolution();
        }
      } break;

      case REVISED_SIMPLEX_TABLEAU: {
        lp_model.ToStandardForm();
        lp_model.ToSlackForm();
        lp_model.ToTableau(COLUMN_ONLY);
        result = lp_model.TableauRevisedSimplexSolve();
        if (result == Result::SOLVED) {
          optimum = lp_model.GetTableauRevisedSimplexOptimum();
          solution = lp_model.GetTableauRevisedSimplexSolution();
        }
      } break;

      case COLUMN_GENERATION: {
        lp_model.ToStandardForm();
        result = lp_model.ColumnGenerationSolve({}, true);
        if (result == Result::SOLVED) {
          optimum = lp_model.GetColumnGenerationOptimum();
          solution = lp_model.GetColumnGenerationSolution();
        }
      } break;

      default:
        break;
    }

    if (result == Result::SOLVED) {
      std::cout << optimum.ToString() << "\n";
      for (auto entry : solution)
        std::cout << entry.first.ToString() << " = " << entry.second.ToString()
                  << "\n";
    } else if (result == Result::NOSOLUTION) {
      std::cout << "No Solution\n";
    } else if (result == Result::UNBOUNDED) {
      std::cout << "Unbounded\n";
    } else {
      std::cout << "Error\n";
    }
  }
  return 0;
}