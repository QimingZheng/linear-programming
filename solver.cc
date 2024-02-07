#include <fstream>
#include <sstream>

#include "ilp.h"
#include "lp.h"
#include "parser.h"

int main(int argc, char **argv) {
  assert(int(-1.5) == -1);
  assert(int(1.5) == 1);
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " input-file\n";
    return -1;
  }
  std::ifstream lpfile(argv[1]);
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
    std::cout << ilp_model.ToString();
  } else {
    lp_model = model;
    std::cout << lp_model.ToString();
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
    lp_model.ToStandardForm();
    lp_model.ToSlackForm();
    auto res = lp_model.SimplexSolve();
    if (res == Result::SOLVED) {
      std::cout << lp_model.GetOptimum().ToString() << "\n";
      for (auto entry : lp_model.GetSolution()) {
        std::cout << entry.first.ToString() << " = " << entry.second.ToString()
                  << "\n";
      }
    }
  }
  return 0;
}