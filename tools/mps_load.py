import sys
import pulp

unified_var_names = {}


def InitializeUnifiedVarNames(variables):
    total = 0
    for var in variables:
        unified_var_names[var] = "x" + str(total)
        total += 1


def float_to_str(x):
    return format(x, "f")


def parseExpression(affine_expr_coeffs, constant):
    ret = ""
    ind = 0
    for item in affine_expr_coeffs:
        var, coeff = item["name"], item["value"]
        if coeff < 0:
            ret += float_to_str(coeff) + "*" + unified_var_names[var]
        else:
            ret += (
                ("+" if ind > 0 else "")
                + float_to_str(coeff)
                + "*"
                + unified_var_names[var]
            )
        ind += 1
    ret += ("+" if ind > 0 else "") + float_to_str(constant)
    return ret


def parseConstraint(con):
    con = con.toDict()
    ret = parseExpression(con["coefficients"], 0.0)
    if con["sense"] == 0:
        ret += "="
    elif con["sense"] == 1:
        ret += ">="
    else:
        ret += "<="
    ret += float_to_str(-con["constant"])
    return ret


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("python3 %s input-mps-file output" % sys.argv[0])
    input = sys.argv[1]
    output = sys.argv[2]
    if len(sys.argv) >= 2:
        variables, instance = pulp.LpProblem.fromMPS(sys.argv[1])
        print(str(len(variables)) + " variables")
        print(str(len(instance.constraints)) + " constraints")
        InitializeUnifiedVarNames(variables)
        with open(output, "w") as out:
            if instance.sense == pulp.LpMinimize:
                out.write("min ")
            else:
                out.write("max ")
            out.write(
                parseExpression(
                    instance.objective.toDict(), instance.objective.constant
                )
                + "\n"
            )
            out.write("st\n")
            for k, con in instance.constraints.items():
                out.write(parseConstraint(con) + "\n")
            for var in instance.variables():
                assert var.cat == "Continuous"
                if var.lowBound != None:
                    out.write(
                        unified_var_names[var.name]
                        + ">="
                        + float_to_str(var.lowBound)
                        + "\n"
                    )
                if var.upBound != None:
                    out.write(
                        unified_var_names[var.name]
                        + "<="
                        + float_to_str(var.upBound)
                        + "\n"
                    )
