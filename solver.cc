#include <fstream>
#include <sstream>

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
LPModel ParseFile(std::string filename) {
  LPModel model;
  std::ifstream lpfile(filename);
  std::string line;
  if (lpfile.is_open()) {
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
      Constraint constraint(FLOAT);
      Expression expression(0.0f);
      OptimizationObject obj(FLOAT);
      bool is_positive = true;
      Num num = 1.0f;
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
            num = 1.0f;
            added = true;
          } break;

          case kNum:
            num = std::stof(field);
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
            num = 1.0f;
          } break;

          default:
            break;
        }
      }
      constraint.SetCompare(num);
      if (is_opt_obj)
        model.SetOptimizationObject(obj);
      else
        model.AddConstraint(constraint);
    }
    lpfile.close();
  } else {
    throw std::runtime_error("Unable to open file");
  }

  return model;
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "usage: " << argv[0] << " input-file\n";
    return -1;
  }
  LPModel model = ParseFile(argv[1]);
  std::cout << model.ToString();
  model.ToStandardForm();
  model.ToRelaxedForm();
  model.Solve();
  std::cout << model.GetOptimum().ToString() << "\n";
  return 0;
}