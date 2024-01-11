# Integer programming

## 割平面法

### Gomory cut


使用单纯形法求解线性问题会产生一组如下形式的方程
:
$x_i+\sum \bar a_{i,j}x_j=\bar b_i$

其中 $x_i$ 是基本变量， $x_j$是非基本变量。重写方程，使整数部分位于等号左边，小数部分位于等号右边：

$x_i+\sum \lfloor \bar a_{i,j} \rfloor x_j - \lfloor \bar b_i \rfloor  = \bar b_i - \lfloor \bar b_i \rfloor - \sum ( \bar a_{i,j} -\lfloor \bar a_{i,j} \rfloor) x_j$

对于任意位于可行域的整数点，等号右边小于 1 ，而等号左边为整数，因此两边共同的取值必然小于或等于 0 。因此不等式

$\bar b_i - \lfloor \bar b_i \rfloor - \sum ( \bar a_{i,j} -\lfloor \bar a_{i,j} \rfloor) x_j \le 0$

对于可行域内的所有整数点必须成立。此外，在基本可行解中，非基本变量都为 0 ，而且基本可行解 x 中如果 $x_i$ 不是整数，

$\bar b_i - \lfloor \bar b_i \rfloor - \sum ( \bar a_{i,j} -\lfloor \bar a_{i,j} \rfloor) x_j = \bar b_i - \lfloor \bar b_i \rfloor > 0$

所以上方的不等式排除了基本可行解，并且是符合需求的一次切割。通过将新的松弛变量 $x_k$ 引入不等式中，新的约束得以加入到线性问题中：

$x_k + \sum (\lfloor \bar a_{i,j} \rfloor - \bar a_{i,j}) x_j = \lfloor \bar b_i \rfloor - \bar b_i,\, x_k \ge 0,\, x_k \in Z$
