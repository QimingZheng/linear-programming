#include "parser.h"

std::vector<Token> Lexer::Scan(std::string input) {
  int cur = 0;
  std::vector<Token> ret;
  const Token kUnknownTk = {Token::UNKNOWN, "", -1, -1};
  Token next_tk = kUnknownTk;
  while (cur < input.size()) {
    char c = input[cur];
    switch (c) {
      case '\n':
        if (next_tk != kUnknownTk) {
          next_tk.lexim =
              input.substr(next_tk.start_index, cur - next_tk.start_index);
          next_tk.end_index = cur;
          ret.push_back(next_tk);
        }
        next_tk = {Token::EOL, "\n", cur, cur + 1};
        ret.push_back(next_tk);
        next_tk = kUnknownTk;
        break;
      case '\0':
      case ' ':
      case '\t':
        if (next_tk != kUnknownTk) {
          next_tk.lexim =
              input.substr(next_tk.start_index, cur - next_tk.start_index);
          next_tk.end_index = cur;
          ret.push_back(next_tk);
          next_tk = kUnknownTk;
        }
        break;
      default: {
        if (IsLetter(c)) {
          if (next_tk == kUnknownTk) {
            next_tk = {Token::VAR, "", cur, -1};
          } else if (next_tk.type == Token::NUM) {
            throw std::runtime_error("cannot concat num + var");
          }
        } else if (IsDigit(c)) {
          if (next_tk == kUnknownTk) {
            next_tk = {Token::NUM, "", cur, -1};
          }
        } else if (c == '.') {
          if (next_tk == kUnknownTk) {
            next_tk = {Token::NUM, "", cur, -1};
          } else {
            assert(next_tk.type == Token::NUM);
          }
        } else if (c == '+' or c == '-' or c == '*' or c == '>' or c == '<' or
                   c == '=' or c == ',') {
          if (next_tk != kUnknownTk) {
            next_tk.lexim =
                input.substr(next_tk.start_index, cur - next_tk.start_index);
            next_tk.end_index = cur;
            ret.push_back(next_tk);
          }
          switch (c) {
            case '+':
              next_tk = {Token::ADD, "+", cur, cur + 1};
              ret.push_back(next_tk);
              break;
            case '-':
              next_tk = {Token::SUB, "-", cur, cur + 1};
              ret.push_back(next_tk);
              break;
            case '*':
              next_tk = {Token::MUL, "*", cur, cur + 1};
              ret.push_back(next_tk);
              break;
            case '>':
              assert(cur + 1 < input.size());
              assert(input[cur + 1] == '=');
              next_tk = {Token::GE, ">=", cur, cur + 2};
              ret.push_back(next_tk);
              cur += 1;
              break;
            case '<':
              assert(cur + 1 < input.size());
              assert(input[cur + 1] == '=');
              next_tk = {Token::LE, "<=", cur, cur + 2};
              ret.push_back(next_tk);
              cur += 1;
              break;
            case '=':
              next_tk = {Token::EQ, "=", cur, cur + 1};
              ret.push_back(next_tk);
              break;
            case ',':
              next_tk = {Token::COMMA, ",", cur, cur + 1};
              ret.push_back(next_tk);
              break;
          }
          next_tk = kUnknownTk;
        } else {
          throw std::runtime_error("Lexing error: un-recognized symbol");
        }
      } break;
    }
    cur += 1;
  }
  if (next_tk != kUnknownTk) {
    next_tk.end_index = input.size();
    next_tk.lexim = input.substr(next_tk.start_index,
                                 next_tk.end_index - next_tk.start_index);
    ret.push_back(next_tk);
  }
  for (auto &tk : ret) {
    if (tk.type == Token::VAR) {
      if (tk.lexim == "MAX" or tk.lexim == "max") {
        tk.type = Token::MAX;
      }
      if (tk.lexim == "MIN" or tk.lexim == "min") {
        tk.type = Token::MIN;
      }
      if (tk.lexim == "ST" or tk.lexim == "st") {
        tk.type = Token::ST;
      }
    }
  }
  return ret;
}

bool Lexer::IsLetter(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c == '_');
}

bool Lexer::IsDigit(char c) { return c >= '0' && c <= '9'; }

OptimizationObject Parser::ParseOptimizationObject(std::vector<Token> tokens) {
  assert(tokens.size() > 0);
  assert(tokens.begin()->type == Token::MAX or
         tokens.begin()->type == Token::MIN);
  OptimizationObject obj(FLOAT);
  obj.opt_type = tokens.begin()->type == Token::MAX ? OptimizationObject::MAX
                                                    : OptimizationObject::MIN;
  std::vector<Token> exp(tokens.begin() + 1, tokens.end());
  obj.expression = ParseExpression(exp);
  return obj;
}
Constraint Parser::ParseConstraint(std::vector<Token> tokens) {
  assert(tokens.size() >= 3);
  assert(tokens[tokens.size() - 2].type == Token::EQ or
         tokens[tokens.size() - 2].type == Token::GE or
         tokens[tokens.size() - 2].type == Token::LE);
  assert(tokens.back().type == Token::NUM);
  Constraint ret(FLOAT);
  ret.equation_type =
      tokens[tokens.size() - 2].type == Token::EQ
          ? Constraint::EQ
          : (tokens[tokens.size() - 2].type == Token::GE ? Constraint::GE
                                                         : Constraint::LE);
  std::vector<Token> exp(tokens.begin(), tokens.begin() + tokens.size() - 2);
  ret.expression = ParseExpression(exp);
  ret.compare = Num(std::stof(tokens.back().lexim));
  return ret;
}
Expression Parser::ParseExpression(std::vector<Token> tokens) {
  std::vector<Token> group;
  Expression ret = Expression(kFloatZero);
  bool sign = true;
  auto parse_unary = [](std::vector<Token> group) -> Expression {
    auto group_sz = group.size();
    assert(group_sz == 1 or group_sz == 3);
    if (group.size() == 1) {
      auto item = *group.begin();
      assert(item.type == Token::NUM or item.type == Token::VAR);
      if (group.begin()->type == Token::NUM) {
        return Expression(Num(std::stof(item.lexim)));
      } else {
        return Expression(Variable(item.lexim));
      }
    }
    if (group.size() == 3) {
      assert(group[0].type == Token::NUM);
      assert(group[1].type == Token::MUL);
      assert(group[2].type == Token::VAR);
      return Num(std::stof(group[0].lexim)) * Variable(group[2].lexim);
    }
    throw std::runtime_error("parse unary error\n");
  };
  int i = 0;
  while (i < tokens.size()) {
    auto tk = tokens[i];
    if (tk.type == Token::ADD or tk.type == Token::SUB) {
      if (i != 0) {
        auto exp = parse_unary(group);
        if (sign)
          ret += exp;
        else
          ret -= exp;
      }
      sign = tk.type == Token::ADD;
      group.clear();
    } else {
      group.push_back(tk);
    }
    i += 1;
  }
  if (group.size() > 0) {
    auto exp = parse_unary(group);
    if (sign)
      ret += exp;
    else
      ret -= exp;
  }

  return ret;
}

Model Parser::Parse(std::vector<Token> tokens) {
  std::vector<std::vector<Token>> lines;
  std::vector<Token> line;
  for (int i = 0; i < tokens.size(); i++) {
    if (tokens[i].type == Token::EOL) {
      lines.push_back(line);
      line.clear();
    } else {
      line.push_back(tokens[i]);
    }
  }
  if (line.size()) lines.push_back(line);
  assert(lines.size() >= 2);
  assert(lines[0].size() >= 2);
  assert(lines[1].size() == 1);
  assert(lines[1][0].type == Token::ST);
  OptimizationObject opt_obj = ParseOptimizationObject(lines[0]);
  // The last line can be an integer constraint.
  auto sz = lines.size();
  std::set<Variable> integers;
  bool is_integer_constraint = false;
  for (auto tk : lines[lines.size() - 1]) {
    if (tk.type == Token::COMMA) {
      is_integer_constraint = true;
      break;
    }
  }
  if (is_integer_constraint) {
    sz -= 1;
    for (auto tk : lines[lines.size() - 1]) {
      if (tk.type == Token::VAR) {
        integers.insert(Variable(tk.lexim, INTEGER));
      }
    }
  }
  for (auto var : integers) {
    Variable float_var(var.variable_name, FLOAT);
    if (opt_obj.expression.GetCoeffOf(float_var) != kFloatZero) {
      opt_obj.expression.SetCoeffOf(var,
                                    opt_obj.expression.GetCoeffOf(float_var));
      opt_obj.expression.SetCoeffOf(float_var, kFloatZero);
    }
  }
  std::vector<Constraint> constraints;
  for (int i = 2; i < sz; i++) {
    auto con = ParseConstraint(lines[i]);
    for (auto var : integers) {
      Variable float_var(var.variable_name, FLOAT);
      if (con.expression.GetCoeffOf(float_var) != kFloatZero) {
        con.expression.SetCoeffOf(var, con.expression.GetCoeffOf(float_var));
        con.expression.SetCoeffOf(float_var, kFloatZero);
      }
    }
    constraints.push_back(con);
  }
  return {constraints, opt_obj};
}

Model Parser::Parse(std::string input) {
  Lexer lexer;
  auto tokens = lexer.Scan(input);
  return Parse(tokens);
}

Model Parser::Parse(std::ifstream &file) {
  if (file.is_open()) {
    std::string input;
    std::string line;
    while (std::getline(file, line)) {
      input += line + "\n";
    }
    file.close();
    return Parse(input);
  } else {
    throw std::runtime_error("Cannot find file\n");
  }
}