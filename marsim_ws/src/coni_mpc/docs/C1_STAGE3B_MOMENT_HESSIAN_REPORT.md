# C1 阶段 3B：矩性质与 Taylor 修正审计

本阶段只增加独立参考实现和数值测试，没有修改 ROS、DSP 发布、HOCBF、MPC、ACADO 或阶段 1 的碰撞核。参考实现是
[`stage3b_moment_hessian_experiment.py`](../scripts/stage3b_moment_hessian_experiment.py)，测试是
[`stage3b_moment_hessian_test.py`](../test/stage3b_moment_hessian_test.py)。

## 1. 形函数矩性质

令 `k=floor(p/Delta)`、`u=p/Delta-k`，其中 `0<=u<1`。现有一维 cubic cardinal B-spline 的四个权重（节点 `k-1,k,k+1,k+2`）为

```text
B0=(1-u)^3/6
B1=(3u^3-6u^2+4)/6
B2=(-3u^3+3u^2+3u+1)/6
B3=u^3/6.
```

直接展开多项式得到

```text
sum_a B_a = 1,
sum_a (k+a) B_a = k+u,
sum_a ((k+a)-(k+u))^2 B_a = 1/3.
```

三维权重是三个一维权重的乘积。因此，在完整支持域（所有 4×4×4 节点都存在）有

```text
Σ_j N_ij = 1,
Σ_j N_ij c_j = p_i,
Σ_j N_ij ||c_j-p_i||² = 3 Delta²/3 = Delta².
```

独立数值测试在 `Delta=1,0.5,0.25,0.13` 和多个非格点上验证了这些式子。浮点误差如下：分割和、二阶矩均约为 `1e-15`，一阶位置误差约为 `1e-16--1e-15`。边界截断不满足这些式子：例如 `p=(0.2,0.3,0.4), Delta=1` 在 8³ 有限网格裁掉支持后，`ΣN=0.8313326062`，一阶误差范数约 `0.0654`，二阶矩为 `0.6266158575`。现有 P2G 对非零支持越过有限网格直接抛出异常，因此矩证明的适用域必须保持为完整支持域，不能把边界裁剪结果用于证明。

## 2. smootherstep 核的全局 Hessian 界

设物理半径 `r>0`，过渡宽度 `delta_sm>0`，`R=r+delta_sm`。现有核为

```text
phi(d)=1,                         d<=r
phi(d)=g((R-d)/delta_sm),         r<d<R
phi(d)=0,                         d>=R
g(x)=6x^5-15x^4+10x^3.
```

在过渡区

```text
g'(x)  = 30 x²(1-x)²,
g''(x) = 60 x(1-x)(1-2x),
phi'(d)  = -g'(x)/delta_sm,
phi''(d) =  g''(x)/delta_sm².
```

对 `f_q(p)=phi(||q-p||)`，令 `d=||q-p||`、`u=(q-p)/d`。过渡区一定有 `d>=r>0`，所以径向 Hessian 没有原点奇异性，且

```text
H = phi''(d) uu^T + (phi'(d)/d)(I-uu^T).
```

其谱范数是两个特征值绝对值的最大值：

```text
|lambda_radial| = |g''(x)|/delta_sm²,
|lambda_tangent| = g'(x)/(delta_sm d).
```

在 `[0,1]` 上，`max|g''|=10/sqrt(3)`，`max g'=15/8`，且过渡区 `d>=r`。因此一个紧的全局界为

```text
L2 = max( 10/(sqrt(3)*delta_sm²), 15/(8*r*delta_sm) ).
```

平台区和支撑外部 Hessian 为零。因为 `g'(0)=g'(1)=g''(0)=g''(1)=0`，内外接缝保持 C²；`r>0` 保证平台到过渡区的接缝不经过径向原点。独立密集扫描 `r=1, delta_sm=0.5` 得到 `23.0940107577`，闭式界为 `23.0940107676`，差值约 `1e-8`。

## 3. Taylor 展开实际给出的符号

对每个粒子和其支持节点，令 `e_ij=c_j-p_i`，在 **`p_i` 处** 展开（这样一阶项使用同一个梯度，矩性质才能消掉它）：

```text
f_q(c_j) = f_q(p_i) + grad f_q(p_i)^T e_ij
           + 1/2 e_ij^T H_q(p_i+theta_ij e_ij)e_ij.
```

乘以 `w_i N_ij` 并对 `j` 求和，`Σ_j N_ij e_ij=0` 消除一阶项。用上面的 `L2` 界可得

```text
|R_grid0 - R_smooth|
    <= (L2/2) Σ_j D_j,
D_j = Σ_i w_i N_ij ||p_i-c_j||².
```

所以由范数界能严格推出的方向是

```text
R_smooth <= R_grid0 + (L2/2) Σ_j D_j chi_j(q).
```

不能推出用户指定的

```text
R_grid0 - (L2/2) Σ_j D_j chi_j(q).
```

原因有两层：第一，Hessian 范数只给绝对值界；第二，smootherstep 过渡区的径向特征值 `phi''` 在 `x<1/2` 为正、在 `x>1/2` 为负，并非全局负半定。因而 Taylor 余项没有可以支持减号的固定符号。

若只讨论可行的 **加号** 参考式，记 `rho=2sqrt(3) Delta`、`R=r+delta_sm`，则可定义一个局部窗：

```text
chi_j(q)=1,                                      ||q-c_j||<=R+rho
chi_j(q)=g((R+rho+s-||q-c_j||)/s),               R+rho<||q-c_j||<R+rho+s
chi_j(q)=0,                                      ||q-c_j||>=R+rho+s,
```

其中任意 `s>0`。它非负、C²、紧支撑；在可能出现 Taylor 余项的区域恒为 1。超过 `R+rho` 后，节点和其全部 cubic 支持粒子的核贡献都严格为零，才允许平滑衰减。因此它能把上述**加号**绝对值界局部化，但不能修复减号。

## 4. 现有核的数值反例

独立参考实现搜索到以下单粒子反例（单粒子已经足以验证符号，且 64 节点求和仍满足一阶矩消除）：

```text
Delta=1, r=1, delta_sm=0.5
p=(0.37, 0.23, -0.19), weight=1
q=(1.405, 0.23, -0.19)
L2=23.0940107676
R_hard=0
R_smooth=0.9969200658
R_grid0=0.3466650778
(L2/2) sum_j D_j = 11.5470053838
R_grid0 - (L2/2) sum_j D_j = -11.2003403060
```

这里 `R_smooth > R_moment`，所以指定的 `R_moment` 不是保守上界。按 `chi_j=1` 的必要区域条件，`q` 位于该粒子所有可能非零余项的局部窗内；令 `chi` 在这些节点上衰减会直接违反“可能有非零 Taylor 余项区域内等于 1”的要求。该反例也不是边界截断造成的。

同一符号错误在四个互不相同的粒子上仍成立：`q_x=1.365` 时，`R_smooth=1.3807740255`，`R_grid0=0.4999866725`，修正量为 `16.1658075373`，减号候选为 `-15.6658208648`。

## 5. 停止决策与验证范围

新增的独立测试命令为：

```bash
python3 -m unittest -v marsim_ws/src/coni_mpc/test/stage3b_moment_hessian_test.py
python3 marsim_ws/src/coni_mpc/scripts/stage3b_moment_hessian_experiment.py --json
```

五项测试（完整支持域矩性质、边界截断警示、Hessian 闭式界、单粒子和四粒子减号反例）均通过。参考脚本还记录了 Python 独立实现的探索性 P2G+矩项+风险构造耗时（`Delta=0.5`，20 个查询步）：256/1024/2048 粒子约为 60--65/208--221/373--393 ms/步；这不是 C++ 3A 实时耗时，也不构成替换依据。

由于严格保守性在第 4 节已经被反例否定，本阶段按要求停止在数学审计和独立参考实现，不伪造 `R_hard <= R_smooth <= R_moment` 对比，不把无效方案接入 P2G、HOCBF、MPC 或 ACADO，也不删除阶段 3A 的 `rho=2sqrt(3)Delta` 方案。建议保留阶段 3A。若后续要继续研究，应单独评估上面给出的**加号**局部化界；它的保守性与 3A 的相对紧致性仍需新的、独立的成对实验确认。
