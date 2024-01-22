## Column Generation

### Definitions

1. Master problem: the original problem with only a subset of variables being considered.
2. Sub problem (or Pricing problem): a new problem created to identify a variable that can improve the objective function of the master problem.

### Algorithm

1. Initialize the master problem and sub problem.
2. Solve the master problem.
3. Search for an improving variable in the sub problem.
    - If found, add this variable into the master problem.
    - Otherwise, the master problem has achieved the optimal.


### Find an improving variable

Suppose the master problem is a minimization problem. Then we want to find a variable that can reduce the cost (objective function) the most (**most negative**). If no such variable (all remaining variables have positive **reduced cost**), then the solution to the current master problem is optimal.


We now detail how and why to compute the reduced cost of the variables. Consider the following linear program in standard form:
$\begin{align}
    &\min_{x}  c^T x  \\
    &\text{subjected to} \\
    & A x = b \\
    & x \in \mathbb{R}^+
\end{align}$

which we will call the primal problem as well as its dual linear program:

$\begin{align}
    &\max_{u}  u^T b  \\
    &\text{subjected to} \\
    & u^T A \leq c \\
    & u \in \mathbb{R}
\end{align}$

Moreover, let $ x^* $ and $ u^* $ be optimal solutions for these two problems which can be provided by any linear solver. These solutions verify the constraints of their linear program and, by duality theorems, have the same value of objective function ($c^T x^* = u^{*T} b$) which we will call $ z^* $. This optimal value is a function of the different coefficients of the primal problem: $ z^* = z^* (c, A, b) $. Note that there exists a dual variable $ u_i^* $ for each constraint of the primal linear model. It is possible to show that an optimal dual variable $ u_i^* $ can be interpreted as the partial derivative of the optimal value $ z^* $ of the objective function with respect to the coefficient $ b_i $ of the right-hand side of the constraints: $u_i^* = \frac{\partial z^*}{\partial b_i}$ or otherwise $u^* = \frac{\partial z^*}{\partial b}$. More simply put, $u_i^*$ indicates by how much increases locally the optimal value of the objective function when the coefficient $b_i$ increases by one unit.

Consider now that a variable $y$ was not considered until then in the primal problem. Note that this is equivalent to saying that the variable $y$ was present in the model but took a zero value. We will now observe the impact on the primal problem of changing the value of $ y $ from $0$ to $\hat{y}$. If $ c_y $ and $A_y$ are respectively the coefficients associated with the variable $ y $ in the objective function and in the constraints then the linear program is modified as follows:

$\begin{align}
    &\min_{x}  c^T x + c_y \hat{y}  \\
    &\text{subjected to} \\
    & A x = b - A_y \hat{y}\\
    & x \in \mathbb{R}^+
\end{align}$

In order to know if it is interesting to add the variable $ y $ to the problem (''i.e'' to let it take a non-zero value), we want to know if the value $z_{\hat{y}}^*$ of the objective function of this new problem decreases as the value $\hat{y}$ of the variable $ y $ increases. In other words, we want to know $\frac{d z_{\hat{y}}^*}{d \hat{y}}$. To do this, note that $z_{\hat{y}}^*$ can be expressed according to the value of the objective function of the initial primal problem: $z_{\hat{y}}^* = c_y \hat{y} + z^*(c, A, b-A_y \hat{y})$. We can then compute the derivative that interests us:

$\begin{align}
    \frac{d z_{\hat{y}}^*}{d \hat{y}} & ~=~ & & \frac{\partial z_{\hat{y}}^*}{\partial \hat{y}} + \frac{d z^*}{d \hat{y}} \\
    & ~=~ & & c_y + \frac{\partial z^*}{\partial c} \frac{d c}{d \hat{y}} + \frac{\partial z^*}{\partial A} \frac{d A}{d \hat{y}} + \frac{\partial z^*}{\partial b} \frac{d b}{d \hat{y}} \\
    & ~=~ & & c_y + \frac{\partial z^*}{\partial b} \frac{d b}{d \hat{y}} \\
    & ~=~ & & c_y + u^* (-A_y) \\
    & ~=~ & & c_y - u^* A_y
\end{align}$

In other words, the impact of changing the value $\hat{y}$ on the value $z_{\hat{y}}^*$ translates into two terms. First, this change directly impacts the objective function and second, the right-hand side of the constraints is modified which has an impact on the optimal variables $x^*$ whose magnitude is measured using the dual variables $u^*$. The derivative $\frac{d z_{\hat{y}}^*}{d \hat{y}}$ is generally called the reduced cost of the variable $y$ and will be denoted by $cr_y$ in the following.


### Reference

1. https://en.wikipedia.org/wiki/Column_generation
2. [Column Generation Method](./column-generation.pdf)
