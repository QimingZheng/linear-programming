#include <fstream>
#include <sstream>

#include "ilp.h"
#include "lp.h"

template <typename Out>
void split(const std::string &s, char delim, Out result) {
  std::istringstream iss(s);
  std::string item;
  while (std::getline(iss, item, delim)) {
    *result++ = item;
  }
}

std::vector<std::string> split(const std::string &s, char delim) {
  std::vector<std::string> elems;
  split(s, delim, std::back_inserter(elems));
  return elems;
}

/* The format of an LP file:
 *  1. objective
 *  2. starting from the second line, a list of constraints, one constraint one
 * line.
 */
bool ParseFile(std::string filename, LPModel &lp_model, ILPModel &ilp_model) {
  std::ifstream lpfile(filename);
  std::string line;
  bool is_ilp = false;
  if (!lpfile.is_open()) {
    throw std::runtime_error("Unable to open file");
  }
  std::getline(lpfile, line);
  if (line == "int") is_ilp = true;
  while (std::getline(lpfile, line)) {
    auto fields = split(line, ' ');
    const int kSymbol = 1, kNum = 2, kCompare = 3, kSign = 4, kMinMax = 5;
    auto type_of_field = [](std::string str) -> int {
      assert(str.length() > 0);
      if (str == "+" or str == "-") return kSign;
      if (str == ">=" or str == "<=" or str == "=") return kCompare;
      if (str[0] >= '0' and str[0] <= '9') {
        return kNum;
      } else {
        if (str == "min" || str == "max") return kMinMax;
        return kSymbol;
      }
    };
    Constraint constraint(is_ilp ? INTEGER : FLOAT);
    Expression expression(is_ilp ? kIntZero : kFloatZero);
    OptimizationObject obj(is_ilp ? INTEGER : FLOAT);
    bool is_positive = true;
    Num num = is_ilp ? kIntOne : kFloatOne;
    bool added = true;
    bool is_opt_obj = false;
    for (auto field : fields) {
      auto type = type_of_field(field);
      switch (type) {
        case kMinMax: {
          is_opt_obj = true;
          obj.expression = expression;
          if (field == "min") {
            obj.SetOptType(OptimizationObject::MIN);
          } else {
            obj.SetOptType(OptimizationObject::MAX);
          }
        } break;

        case kSymbol: {
          Variable x(field);
          expression.SetCoeffOf(x, is_positive ? num : -num);
          is_positive = true;
          num = is_ilp ? kIntOne : kFloatOne;
          added = true;
        } break;

        case kNum:
          num = is_ilp ? Num(std::stoi(field)) : Num(std::stof(field));
          break;

        case kCompare: {
          if (!added) expression.constant = (is_positive ? num : -num);
          constraint.expression = expression;
          if (field == "=") {
            constraint.SetEquationType(Constraint::Type::EQ);
          } else if (field == ">=") {
            constraint.SetEquationType(Constraint::Type::GE);
          } else if (field == "<=") {
            constraint.SetEquationType(Constraint::Type::LE);
          }
        } break;

        case kSign: {
          if (!added) expression.constant = (is_positive ? num : -num);
          added = false;
          if (field == "+") {
            is_positive = true;
          } else {
            is_positive = false;
          }
          num = is_ilp ? kIntOne : kFloatOne;
        } break;

        default:
          break;
      }
    }
    constraint.SetCompare(num);
    if (is_opt_obj) {
      if (is_ilp)
        ilp_model.SetOptimizationObject(obj);
      else
        lp_model.SetOptimizationObject(obj);
    } else {
      if (is_ilp)
        ilp_model.AddConstraint(constraint);
      else
        lp_model.AddConstraint(constraint);
    }
  }
  lpfile.close();
  return is_ilp;
}

int main(int argc, char **argv) {
  assert(int(-1.5) == -1);
  assert(int(1.5) == 1);
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " input-file\n";
    return -1;
  }
  LPModel lp_model;
  ILPModel ilp_model;
  bool is_ilp = ParseFile(argv[1], lp_model, ilp_model);
  if (is_ilp)
    std::cout << ilp_model.ToString();
  else
    std::cout << lp_model.ToString();

  if (is_ilp) {
    auto res = ilp_model.CuttingPlaneSolve();
    if (res == ILPModel::Result::SOLVED) {
      std::cout << ilp_model.GetOptimum().ToString() << "\n";
      for (auto entry : ilp_model.GetSolution()) {
        std::cout << entry.first.ToString() << " = " << entry.second.ToString()
                  << "\n";
      }
    }
  } else {
    lp_model.ToStandardForm();
    lp_model.ToSlackForm();
    auto res = lp_model.Solve();
    if (res == LPModel::Result::SOLVED) {
      std::cout << lp_model.GetOptimum().ToString() << "\n";
      for (auto entry : lp_model.GetSolution()) {
        std::cout << entry.first.ToString() << " = " << entry.second.ToString()
                  << "\n";
      }
    }
  }
  return 0;
}